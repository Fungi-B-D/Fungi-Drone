use super::check::CheckError;
use super::helper::{check, generate, header};
use crossbeam_channel::{select_biased, Receiver, RecvError, Sender, TrySendError};
use rand::{Rng, RngCore};
use rand_xoshiro::rand_core::SeedableRng;
use rand_xoshiro::Xoshiro256PlusPlus;
use std::collections::{HashMap, HashSet};
use wg_2024::controller::{DroneCommand, DroneEvent};
use wg_2024::drone::Drone;
use wg_2024::network::{NodeId, SourceRoutingHeader};
use wg_2024::packet::{FloodRequest, NackType, NodeType, Packet, PacketType};

#[derive(Debug)]
pub struct FungiDrone {
    pub(crate) seen_flood_ids: HashSet<(u64, u8)>, //Hashset<(FloodId, InitiatorId)>
    pub(crate) id: NodeId,
    pub(crate) controller_send: Sender<wg_2024::controller::DroneEvent>,
    pub(crate) controller_recv: Receiver<DroneCommand>,
    pub(crate) packet_recv: Receiver<Packet>,
    pub(crate) packet_send: HashMap<NodeId, Sender<Packet>>,
    pub(crate) pdr: f32,
}

enum CommandResult {
    Break,
    Continue,
    NoController,
}

impl Drone for FungiDrone {
    fn new(
        id: NodeId,
        controller_send: Sender<wg_2024::controller::DroneEvent>,
        controller_recv: Receiver<DroneCommand>,
        packet_recv: Receiver<Packet>,
        packet_send: HashMap<NodeId, Sender<Packet>>,
        pdr: f32,
    ) -> Self {
        Self {
            id,
            controller_send,
            controller_recv,
            packet_recv,
            packet_send,
            pdr,
            seen_flood_ids: HashSet::new(),
        }
    }

    fn run(&mut self) {
        loop {
            select_biased! {
                recv(self.controller_recv) -> command_res => {
                    match self.handle_command_internal(command_res){
                        CommandResult::NoController => println!("The simulation controller no longer has a sender!"),
                        CommandResult::Continue => continue,
                        CommandResult::Break => break,
                    }
                }
                recv(self.packet_recv) -> packet_res => {
                    self.handle_packet_internal(packet_res);
                }
            }
        }
    }
}

impl FungiDrone {
    fn handle_command_internal(
        &mut self,
        command_res: Result<DroneCommand, RecvError>,
    ) -> CommandResult {
        if let Ok(msg) = command_res {
            match msg {
                DroneCommand::AddSender(node_id, channel_sender) => {
                    self.packet_send.insert(node_id, channel_sender);
                    return CommandResult::Continue;
                }
                DroneCommand::Crash => {
                    self.crash_behaviour();
                    return CommandResult::Break;
                }
                DroneCommand::SetPacketDropRate(pdr) => {
                    self.pdr = pdr;
                    return CommandResult::Continue;
                }
                DroneCommand::RemoveSender(node_id) => {
                    self.packet_send.remove(&node_id);
                    return CommandResult::Continue;
                }
            }
        }
        CommandResult::NoController
    }

    fn handle_packet_internal(&mut self, packet_res: Result<Packet, RecvError>) {
        if let Ok(msg) = packet_res {
            if let PacketType::FloodRequest(flood_request) = msg.pack_type {
                self.receive_flood_request(flood_request, msg.session_id);
            } else {
                let res = self.check_packet(msg);
                if let Some(pack_ready) = self.handle_check_result(res) {
                    let sender_res = self.get_send_info(&pack_ready);
                    if sender_res.is_none() {
                        return;
                    }

                    let (id, sender) = sender_res.unwrap();

                    self.log_action(pack_ready.clone(), self.is_dropped(pack_ready.clone()));
                    self.send(pack_ready, id, sender);
                }
            }
        } else {
            println!("The drone has no senders, but is not in crash behaviour!");
        }
    }

    fn check_packet(&mut self, packet: Packet) -> Result<Packet, CheckError> {
        // If checks are ok pass ownership of packet back

        let mut packet = check::id_matches_hop(packet, &self)?;

        header::increment_index(&mut packet.routing_header);

        packet = check::destination_is_drone(packet)?;

        packet = check::message_drop(packet, &self)?;

        packet = check::not_neighbor(packet, &self)?;

        Ok(packet)
    }

    fn handle_check_result(&self, res: Result<Packet, CheckError>) -> Option<Packet> {
        if let Ok(p) = res {
            return Some(p);
        }

        match res.unwrap_err() {
            CheckError::MustShortcut(err) => {
                self.send_controller(DroneEvent::ControllerShortcut(err));
                return None;
            }
            CheckError::SendNack(err) => return Some(err),
        }
    }

    fn crash_behaviour(&mut self) {
        while let Ok(mut packet) = self.packet_recv.recv() {
            match packet.pack_type.clone() {
                PacketType::MsgFragment(f) => {
                    header::increment_index(&mut packet.routing_header);
                    self.handle_send_error(packet, self.id, f.fragment_index);
                }
                PacketType::Ack(_) | PacketType::Nack(_) | PacketType::FloodResponse(_) => {
                    let res = self.check_packet(packet);
                    if let Some(pack_ready) = self.handle_check_result(res) {
                        let sender_res = self.get_send_info(&pack_ready);

                        if sender_res.is_none() {
                            return;
                        }

                        let (id, sender) = sender_res.unwrap();
                        self.log_action(pack_ready.clone(), self.is_dropped(pack_ready.clone()));
                        self.send(pack_ready, id, sender);
                    }
                }
                PacketType::FloodRequest(_) => return,
            };
        }
    }

    fn receive_flood_request(&mut self, mut flood_req: FloodRequest, session_id: u64) {
        if self.id == 2 {}

        if self
            .seen_flood_ids
            .contains(&(flood_req.flood_id, flood_req.initiator_id))
        {
            flood_req.path_trace.push((self.id, NodeType::Drone));
            let response = generate::flood_response(self.id, flood_req, session_id);
            let (next_id, p_sender) = self.get_send_info(&response).unwrap();

            self.send(response, next_id, p_sender);
            return;
        }

        self.seen_flood_ids
            .insert((flood_req.flood_id, flood_req.initiator_id));
        let node_before = flood_req.path_trace.last().cloned().unwrap().0;
        flood_req.path_trace.push((self.id, NodeType::Drone));

        if self.packet_send.len() == 1 {
            let response = generate::flood_response(self.id, flood_req, session_id);
            let (next_id, p_sender) = self.get_send_info(&response).unwrap();
            self.send(response, next_id, p_sender);
            return;
        }

        let empty_header = SourceRoutingHeader {
            hop_index: 0,
            hops: Vec::new(),
        };

        let request = Packet {
            pack_type: PacketType::FloodRequest(flood_req),
            routing_header: empty_header,
            session_id,
        };

        for (neighbor_id, sender) in self.packet_send.clone() {
            if neighbor_id != node_before {
                self.send(request.clone(), neighbor_id, sender);
            }
        }
    }

    fn send(&mut self, p: Packet, next_id: u8, p_sender: Sender<Packet>) {
        let res = p_sender.try_send(p);

        if let Err(send_err) = res {
            match send_err {
                TrySendError::Full(_) => {
                    println!("Channel : {:?} is full!", p_sender);
                    panic!("The sender channel is full");
                }
                TrySendError::Disconnected(msg) => match &msg.pack_type.clone() {
                    PacketType::MsgFragment(f) => {
                        self.handle_send_error(msg, next_id, f.fragment_index)
                    }
                    PacketType::Ack(_) | PacketType::Nack(_) | PacketType::FloodResponse(_) => {
                        self.send_controller(DroneEvent::ControllerShortcut(msg))
                    }
                    PacketType::FloodRequest(_) => return,
                },
            }
        }
    }

    fn handle_send_error(&mut self, p: Packet, next_id: u8, f_index: u64) {
        let err_p = generate::route_error(p.routing_header, p.session_id, next_id, f_index);
        if let Some((err_id, err_sender)) = self.get_send_info(&err_p) {
            self.send(err_p, err_id, err_sender);
        }
    }

    fn get_send_info(&self, p: &Packet) -> Option<(u8, Sender<Packet>)> {
        // Since this packet has gone through not neighbor check we can unwrap
        let hop_id = header::get_hop(&p.routing_header);

        if hop_id.is_none() {
            println!("The hop index is out of bounds!");
            self.send_controller(DroneEvent::ControllerShortcut(p.clone()));
            return None;
        }

        let id = hop_id.unwrap();
        let sender_res = self.packet_send.get(&id);

        if sender_res.is_none() {
            println!("The sender is not in the hashmap!");
            self.send_controller(DroneEvent::ControllerShortcut(p.clone()));
            return None;
        }

        return Some((id, self.packet_send.get(&id).unwrap().clone()));
    }

    /// Decide if a package should be dropped according to the probability in [`DroneOptions::pdr`].
    /// The random number is generated using `Xoshiro 256++`, a random library used to generate more unpredictable pseudo-random numbers.
    pub(crate) fn dropped(&self) -> bool {
        let xoshiro_seed = rand::thread_rng().next_u64();
        let mut xoshiro = Xoshiro256PlusPlus::seed_from_u64(xoshiro_seed);
        let random_value = xoshiro.gen_range(0.0..1.0);
        random_value < self.pdr as f64
    }

    fn is_dropped(&self, packet: Packet) -> bool {
        matches!(packet.pack_type, PacketType::Nack(ref n) if matches!(n.nack_type,NackType::Dropped))
    }

    fn send_controller(&self, event: DroneEvent) {
        let res = self.controller_send.try_send(event);
        if let Err(send_err) = res {
            // an error in this channel is an unrecoverable fail of the network
            println!(
                "Drone: {} no longer has access to simulation controller!",
                self.id
            );
            panic!(
                "Channel to Simulation Controller Failed!\nsend_controller error: {:?}",
                send_err
            );
        }
    }

    fn log_action(&self, packet: Packet, dropped: bool) {
        let packet_to_send = match dropped {
            true => DroneEvent::PacketDropped(packet),
            false => DroneEvent::PacketSent(packet),
        };
        self.send_controller(packet_to_send);
    }
}
