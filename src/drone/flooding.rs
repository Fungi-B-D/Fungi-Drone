use wg_2024::network::SourceRoutingHeader;
use wg_2024::packet::{FloodRequest, NodeType, Packet, PacketType};

use super::{generate, FungiDrone};

impl FungiDrone {
    /// Handles the flood request logic:
    /// There are 3 paths:
    /// - The drone has seen the flood id and sends back a flood response
    /// - The drone has not seen the id but has no neighbors so sends back a flood response
    /// - The drone has not seen the id and forwards it to all neighbors besides the previous sender
    pub(super) fn receive_flood_request(&mut self, mut flood_req: FloodRequest, session_id: u64) {
        // Check if the flood has been seen

        if self
            .seen_flood_ids
            .contains(&(flood_req.flood_id, flood_req.initiator_id))
        {
            flood_req.path_trace.push((self.id, NodeType::Drone));
            let response = generate::flood_response(self.id, flood_req, session_id);
            let (response, next_id, p_sender) = self.get_send_info(response).unwrap();

            self.forward(response, next_id, p_sender);
            return;
        }

        self.seen_flood_ids
            .insert((flood_req.flood_id, flood_req.initiator_id));

        let node_before = flood_req.path_trace.last().cloned().unwrap().0;
        flood_req.path_trace.push((self.id, NodeType::Drone));

        // Check if there are neighbors besides the one who sent the request
        // The only case in which you have no one to forward the request to
        // is when you have only 1 neighbor

        if self.packet_send.len() == 1 {
            let response = generate::flood_response(self.id, flood_req, session_id);
            let (response, next_id, p_sender) = self.get_send_info(response).unwrap();
            self.forward(response, next_id, p_sender);
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
                self.forward(request.clone(), neighbor_id, sender);
            }
        }
    }
}
