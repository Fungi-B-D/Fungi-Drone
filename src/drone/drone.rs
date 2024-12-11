use super::helper::{generate, header};
use crossbeam_channel::{select_biased, Receiver, RecvError, Sender, TrySendError};
use rand::{Rng, RngCore};
use rand_xoshiro::rand_core::SeedableRng;
use rand_xoshiro::Xoshiro256PlusPlus;
use std::collections::{HashMap, HashSet};
use wg_2024::controller::{DroneCommand, DroneEvent};
use wg_2024::drone::Drone;
use wg_2024::network::NodeId;
use wg_2024::packet::{NackType, Packet, PacketType};

#[derive(Debug)]
pub struct FungiDrone {
    pub(super) seen_flood_ids: HashSet<(u64, u8)>, //Hashset<(FloodId, InitiatorId)>
    pub(super) id: NodeId,
    pub(super) controller_send: Sender<wg_2024::controller::DroneEvent>,
    pub(super) controller_recv: Receiver<DroneCommand>,
    pub(super) packet_recv: Receiver<Packet>,
    pub(super) packet_send: HashMap<NodeId, Sender<Packet>>,
    pub(super) pdr: f32,
    pub(super) debug_print: bool,
    pub(super) debug_shortcut: bool,
}

pub(super) enum CommandResult {
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
            debug_print: false,
            debug_shortcut: false,
        }
    }

    fn run(&mut self) {
        loop {
            select_biased! {
                recv(self.controller_recv) -> command_res => {
                    match self.handle_command_internal(command_res){
                        CommandResult::NoController => self.debug("The simulation controller no longer has access to this drone",None),
                        CommandResult::Continue => continue,
                        CommandResult::Break => break,
                    }
                },
                recv(self.packet_recv) -> packet_res => {
                  if let Ok(msg) = packet_res {
                    self.handle_packet_internal(msg);
                  } else {
                    self.debug("No senders, but not in crash behaviour",None);
                  }
                }
            }
        }
    }
}

impl FungiDrone {
    /// Controls the receiving of a DroneCommand
    ///
    /// ## Arguments
    /// - `command_res`: The result of listening to the simulation controller, it contains the command
    ///
    /// ## Returns
    /// The action which the drone should do after having dealt with the command
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

    /// Controls the receiving of a Packet
    /// ## Arguments
    /// - `packet_res`: The result of listening to the drone's own Receiver<Packet> , it contains the Packet received
    fn handle_packet_internal(&mut self, msg: Packet) {
        if let PacketType::FloodRequest(flood_request) = msg.pack_type {
            self.receive_flood_request(flood_request, msg.session_id);
        } else {
            let res = self.check_packet(msg);
            if let Some(pack_ready) = self.handle_check_result(res) {
                let sender_res = self.get_send_info(pack_ready); // fine to pass ownership

                if sender_res.is_none() {
                    return;
                }

                let (pack_ready, id, sender) = sender_res.unwrap();

                self.log_action(pack_ready.clone(), self.is_dropped(&pack_ready));
                self.forward(pack_ready, id, sender);
            }
        }
    }

    /// Forwards a packet to the next drone
    ///
    /// ## Arguments
    /// - `p`: Packet to be forwarded
    /// - `next_id`: The idea of the drone to which the packet should be sent
    /// - `p_sender`: The sender of the next drone's channel
    pub(super) fn forward(&mut self, p: Packet, next_id: u8, p_sender: Sender<Packet>) {
        let res = p_sender.try_send(p);

        if res.is_ok() {
            return;
        }

        match res.unwrap_err() {
            TrySendError::Full(msg) => {
                self.debug("The next node's channel is full", Some(msg));
                return;
            }
            TrySendError::Disconnected(msg) => match &msg.pack_type {
                PacketType::MsgFragment(_) => self.handle_send_error(msg, next_id),
                PacketType::Ack(_) | PacketType::Nack(_) | PacketType::FloodResponse(_) => {
                    self.send_controller(DroneEvent::ControllerShortcut(msg))
                }
                PacketType::FloodRequest(_) => return,
            },
        }
    }

    /// Sends back an error in routing containing the crashed drone id
    /// and the fragment index of the Packet
    ///
    /// ## Arguments
    ///
    /// - `p`: Packet to sent
    /// - `next_id`: the id of the crashed drone.
    /// - `f_index`: the fragment index of the current MsgFragment
    pub(super) fn handle_send_error(&mut self, p: Packet, next_id: u8) {
        if let PacketType::MsgFragment(f) = p.pack_type {
            let err_p =
                generate::route_error(p.routing_header, p.session_id, next_id, f.fragment_index);

            if let Some((err_p, err_id, err_sender)) = self.get_send_info(err_p) {
                // fine to pass ownership
                self.forward(err_p, err_id, err_sender);
            }
        }
    }

    /// Gets the sender of a packet which is ready to be sent
    ///
    /// ## Arguments
    /// - `p`: Packet to be sent
    pub(super) fn get_send_info(&self, p: Packet) -> Option<(Packet, u8, Sender<Packet>)> {
        let hop_id = header::get_hop(&p.routing_header);

        if hop_id.is_none() {
            self.debug("The hop index is out of bounds", Some(p));
            return None;
        }

        let id = hop_id.unwrap();
        let sender_res = self.packet_send.get(&id);

        if sender_res.is_none() {
            self.debug("The sender is not in the hashmap", Some(p));
            return None;
        }

        return Some((p, id, self.packet_send.get(&id).unwrap().clone()));
    }

    /// Decides if a package should be dropped according to the probability in [`DroneOptions::pdr`].
    /// The random number is generated using `Xoshiro 256++`, a library used to generate more unpredictable pseudo-random numbers.
    pub(super) fn dropped(&self) -> bool {
        let xoshiro_seed = rand::thread_rng().next_u64();
        let mut xoshiro = Xoshiro256PlusPlus::seed_from_u64(xoshiro_seed);
        let random_value = xoshiro.gen_range(0.0..1.0);
        random_value < self.pdr as f64
    }

    /// Pattern matches to check if a packet has been dropped
    pub(super) fn is_dropped(&self, packet: &Packet) -> bool {
        matches!(packet.pack_type, PacketType::Nack(ref n) if matches!(n.nack_type,NackType::Dropped))
    }

    /// Decides whether to send a packet dropped or packet sent event to the controller
    ///
    /// ## Arguments
    ///
    /// -`packet`: The packet to be sent in the event
    /// -`dropped`: Whether or not the packet has been dropped
    pub(super) fn log_action(&self, packet: Packet, dropped: bool) {
        let packet_to_send = match dropped {
            true => DroneEvent::PacketDropped(packet),
            false => DroneEvent::PacketSent(packet),
        };
        self.send_controller(packet_to_send);
    }

    /// Sends an event packet to the simulation controller
    ///
    /// ## Arguments
    /// - `event`: The event object to be sent
    pub(super) fn send_controller(&self, event: DroneEvent) {
        let res = self.controller_send.try_send(event);
        if let Err(_) = res {
            self.debug("no longer has access to simulation controller!", None);
        }
    }
}
