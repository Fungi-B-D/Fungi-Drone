use check::CheckError;
use wg_2024::{controller::DroneEvent, packet::Packet};

use super::{header, FungiDrone};

impl FungiDrone {
    /// Ensure a packet is correct before forwarding
    /// Returns ```Ok(Packet)``` to give back ownership if packet is valid.
    /// Returns ```Err(CheckError)``` to express error response
    pub(super) fn check_packet(&mut self, packet: Packet) -> Result<Packet, CheckError> {
        let mut packet = check::id_matches_hop(packet, self)?;

        header::increment_index(&mut packet.routing_header);

        packet = check::destination_is_drone(packet)?;

        packet = check::message_drop(packet, self)?;

        packet = check::not_neighbor(packet, self)?;

        Ok(packet)
    }

    /// Control next action based on check result.
    ///
    /// Returns ```Some(Packet)``` if packet is ready to be forwarded
    /// Returns ```None``` if packet was sent to the controller
    pub(super) fn handle_check_result(&self, res: Result<Packet, CheckError>) -> Option<Packet> {
        if let Ok(p) = res {
            return Some(p);
        }

        match res.unwrap_err() {
            CheckError::MustShortcut(err) => {
                self.send_controller(DroneEvent::ControllerShortcut(err));
                None
            }
            CheckError::SendNack(err) => Some(err),
            CheckError::Debug => None,
            CheckError::Dropped(packet) => { self.log_action(packet, true); return None; },
        }
    }
}

mod check {
    use crate::drone::{generate, header};

    use super::super::drone::FungiDrone;
    use wg_2024::packet::{Packet, PacketType};

    #[derive(Debug)]
    pub enum CheckError {
        MustShortcut(Packet),
        SendNack(Packet),
        Dropped(Packet),
        Debug,
    }

    /// Checks if the current drone matches the hop index position of the hop vector
    /// ## Arguments
    /// - `p`: The packet to be checked
    /// - `d`: The current drone
    pub fn id_matches_hop(mut p: Packet, d: &FungiDrone) -> Result<Packet, CheckError> {
        let hop_res = header::get_hop(&p.routing_header);
        if hop_res.is_none() {
            d.debug("Hop index beyond hops length", Some(p));
            return Err(CheckError::Debug);
        }

        let hop = hop_res.unwrap();

        if d.id == hop {
            return Ok(p);
        }

        if let PacketType::MsgFragment(f) = p.pack_type {
            header::increment_index(&mut p.routing_header);

            let err_p =
                generate::unexpected(p.routing_header, p.session_id, d.id, f.fragment_index);
            return Err(CheckError::SendNack(err_p));
        }

        Err(CheckError::MustShortcut(p))
    }

    /// Checks if the current drone is the last hop
    /// ## Arguments
    /// - `p`: The packet to be checked
    pub fn destination_is_drone(p: Packet) -> Result<Packet, CheckError> {
        if p.routing_header.hop_index == p.routing_header.hops.len() {
            if let PacketType::MsgFragment(f) = p.pack_type {
                let err_p =
                    generate::destination_drone(p.routing_header, p.session_id, f.fragment_index);
                return Err(CheckError::SendNack(err_p));
            }
            return Err(CheckError::MustShortcut(p));
        }
        Ok(p)
    }

    /// Rolls to see if packet is dropped
    /// ## Arguments
    /// - `p`: The packet to be checked
    /// - `d`: The current drone
    pub fn message_drop(p: Packet, d: &FungiDrone) -> Result<Packet, CheckError> {
        // We pattern match and then unwrap with if let to avoid cloning
        if !matches!(&p.pack_type, PacketType::MsgFragment(_)) {
            return Ok(p);
        }
        if FungiDrone::dropped(d) {
            return Err(CheckError::Dropped(p));
        }
        Ok(p)
    }

    /// Rolls to see if packet is dropped
    /// ## Arguments
    /// - `p`: The packet to be checked
    /// - `d`: The current drone
    pub fn not_neighbor(p: Packet, d: &FungiDrone) -> Result<Packet, CheckError> {
        if let Some(next_hop) = header::get_hop(&p.routing_header) {
            if !d.packet_send.contains_key(&next_hop) {
                if let PacketType::MsgFragment(msg) = p.pack_type {
                    let err_p = generate::route_error(
                        p.routing_header,
                        p.session_id,
                        next_hop,
                        msg.fragment_index,
                    );
                    return Err(CheckError::SendNack(err_p));
                }

                return Err(CheckError::MustShortcut(p));
            }
            return Ok(p);
        }
        d.debug("hop_index beyond hops length", Some(p));
        Err(CheckError::Debug)
    }
}
