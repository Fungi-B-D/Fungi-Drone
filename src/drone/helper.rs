/// functions to interact with SourceRoutingHeader
pub(super) mod header {
    use wg_2024::network::{NodeId, SourceRoutingHeader};

    pub fn get_hop(h: &SourceRoutingHeader) -> Option<NodeId> {
        h.hops.get(h.hop_index).cloned()
    }

    pub fn increment_index(h: &mut SourceRoutingHeader) {
        h.hop_index += 1;
    }
}

/// These functions generate Nack packets based on the original packet.
/// The returned packet is ready to be sent
pub(super) mod generate {
    use super::paths::{self, flooding_response_path};
    use wg_2024::packet::NackType;
    use wg_2024::{
        network::SourceRoutingHeader,
        packet::{FloodRequest, FloodResponse, Fragment, Nack, Packet, PacketType},
    };

    fn get_reversed_routing_header(routing_header: SourceRoutingHeader) -> SourceRoutingHeader {
        let reversed_route = paths::backwards_path(routing_header.hops, routing_header.hop_index);

        
        SourceRoutingHeader {
            hop_index: 1,
            hops: reversed_route,
        }
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

        let mut corrected_header = get_reversed_routing_header(routing_header);

        if let Some(first) = corrected_header.hops.get_mut(0) {
          *first = drone_id;
        }

        Packet {
            pack_type: PacketType::Nack(unexpected_nack),
            routing_header: corrected_header,
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

        let initiator = flood_req.initiator_id;

        let response = FloodResponse {
            flood_id: flood_req.flood_id,
            path_trace: flood_req.path_trace,
        };

        let packet_header = SourceRoutingHeader {
            hop_index: 1,
            hops: flooding_response_path(node_id, &response.path_trace,initiator),
        };

        Packet {
            pack_type: PacketType::FloodResponse(response),
            routing_header: packet_header,
            session_id,
        }
    }
}
/// Functions used to get paths based on:
/// - FloodRequest/Response: path_trace
/// - Other: route_header
pub(super) mod paths {
    use wg_2024::{network::NodeId, packet::NodeType};

    pub fn backwards_path(mut hops: Vec<NodeId>, hop_index: usize) -> Vec<NodeId> {
        hops.truncate(hop_index);
        hops.reverse();
        hops
    }

    pub fn flooding_response_path(search_id: u8, path_trace: &Vec<(u8, NodeType)>, initiator_id: NodeId) -> Vec<u8> {
      let mut r_path = path_trace
          .into_iter()
          .scan(false, |found, (node_id, _)| {
              if *found == true {
                  return None;
              }
              if *node_id == search_id {
                  *found = true;
              }
              Some(*node_id)
          })
          .collect::<Vec<u8>>();

      r_path.reverse();
      if matches!(r_path.last(),Some(last_el) if *last_el != initiator_id) {
        r_path.push(initiator_id);
      }

      r_path
  }
}
