pub mod header {
    use wg_2024::network::{NodeId, SourceRoutingHeader};

    pub fn get_hop(h: &SourceRoutingHeader) -> Option<NodeId> {
        h.hops.get(h.hop_index).cloned()
    }

    pub fn increment_index(h: &mut SourceRoutingHeader) {
        h.hop_index += 1;
    }
}

pub mod check {
    use super::super::drone::FungiDrone;
    use wg_2024::packet::{Packet, PacketType};

    use super::{generate, header};

    #[derive(Debug)]
    pub enum CheckError {
        MustShortcut(Packet),
        SendNack(Packet),
    }

    // For all these checks if the packet original packet is a message fragment we send a nack in reverse
    // If it was already a Nack/Ack/FloodResponse we need to send it to the sim controller

    pub fn id_matches_hop(mut p: Packet, d: &FungiDrone) -> Result<Packet, CheckError> {
        if let Some(hop) = header::get_hop(&p.routing_header) {
            if d.id != hop {
                if let PacketType::MsgFragment(f) = p.pack_type {
                    header::increment_index(&mut p.routing_header);

                    let err_p = generate::unexpected(
                        p.routing_header,
                        p.session_id,
                        d.id,
                        f.fragment_index,
                    );
                    return Err(CheckError::SendNack(err_p));
                }
            }

            return Ok(p);
        }
        println!("Error: Id does not match hop_index, hop index beyond hops length");
        return Err(CheckError::MustShortcut(p));
    }

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

    pub fn message_drop(p: Packet, d: &FungiDrone) -> Result<Packet, CheckError> {
        if let PacketType::MsgFragment(msg_framgent) = p.clone().pack_type {
            if FungiDrone::dropped(d) {
                let err_p = generate::dropped_packet(p.routing_header, p.session_id, msg_framgent);
                return Err(CheckError::SendNack(err_p));
            }
        }
        Ok(p)
    }

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
        println!("Error: Next hop not neighbor, hop_index beyond hops length");
        Err(CheckError::MustShortcut(p))
    }
}

pub mod generate {
    use super::paths::{self, flooding_response_path};
    use wg_2024::packet::NackType;
    use wg_2024::{
        network::SourceRoutingHeader,
        packet::{FloodRequest, FloodResponse, Fragment, Nack, Packet, PacketType},
    };

    fn get_reversed_routing_header(routing_header: SourceRoutingHeader) -> SourceRoutingHeader {
        let reversed_route = paths::backwards_path(routing_header.hops, routing_header.hop_index);

        let reversed_routing_header = SourceRoutingHeader {
            hop_index: 1,
            hops: reversed_route,
        };
        reversed_routing_header
    }

    pub fn unexpected(
        routing_header: SourceRoutingHeader,
        session_id: u64,
        drone_id: u8,
        f_index: u64,
    ) -> Packet {
        let unexpected_nack = Nack {
            fragment_index: f_index,
            nack_type: NackType::UnexpectedRecipient(drone_id),
        };

        Packet {
            pack_type: PacketType::Nack(unexpected_nack),
            routing_header: get_reversed_routing_header(routing_header),
            session_id,
        }
    }

    pub fn dropped_packet(
        routing_header: SourceRoutingHeader,
        session_id: u64,
        msg_fragment: Fragment,
    ) -> Packet {
        let dropped_nack = Nack {
            fragment_index: msg_fragment.fragment_index,
            nack_type: NackType::Dropped,
        };

        Packet {
            pack_type: PacketType::Nack(dropped_nack),
            routing_header: get_reversed_routing_header(routing_header),
            session_id,
        }
    }

    pub fn destination_drone(
        routing_header: SourceRoutingHeader,
        session_id: u64,
        f_index: u64,
    ) -> Packet {
        let destination_nack = Nack {
            fragment_index: f_index,
            nack_type: NackType::DestinationIsDrone,
        };

        Packet {
            pack_type: PacketType::Nack(destination_nack),
            routing_header: get_reversed_routing_header(routing_header),
            session_id,
        }
    }

    pub fn route_error(
        routing_header: SourceRoutingHeader,
        session_id: u64,
        crash_id: u8,
        f_index: u64,
    ) -> Packet {
        let route_nack = Nack {
            fragment_index: f_index,
            nack_type: NackType::ErrorInRouting(crash_id),
        };

        Packet {
            pack_type: PacketType::Nack(route_nack),
            routing_header: get_reversed_routing_header(routing_header),
            session_id,
        }
    }

    pub fn flood_response(node_id: u8, flood_req: FloodRequest, session_id: u64) -> Packet {
        let unzip_path = paths::unzip_path(&flood_req.path_trace);

        let response = FloodResponse {
            flood_id: flood_req.flood_id,
            path_trace: flood_req.path_trace,
        };

        let packet_header = SourceRoutingHeader {
            hop_index: 1,
            hops: flooding_response_path(node_id, unzip_path),
        };

        Packet {
            pack_type: PacketType::FloodResponse(response),
            routing_header: packet_header,
            session_id,
        }
    }
}

pub mod paths {
    use wg_2024::{network::NodeId, packet::NodeType};

    pub fn backwards_path(mut hops: Vec<NodeId>, hop_index: usize) -> Vec<NodeId> {
        hops.truncate(hop_index);
        hops.reverse();
        hops
    }

    pub fn unzip_path(path_trace: &Vec<(u8, NodeType)>) -> Vec<u8> {
        let (l, _): (Vec<u8>, Vec<NodeType>) = path_trace.iter().cloned().unzip();
        l
    }

    pub fn flooding_response_path(search_id: u8, path_trace: Vec<u8>) -> Vec<u8> {
        let mut r_path = path_trace
            .into_iter()
            .scan(false, |found, node_id| {
                if *found == true {
                    return None;
                }
                if node_id == search_id {
                    *found = true;
                }
                Some(node_id)
            })
            .collect::<Vec<u8>>();
        r_path.reverse();
        r_path
    }
}
