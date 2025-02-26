use super::{header, FungiDrone};
use wg_2024::packet::{Packet, PacketType};

impl FungiDrone {
    /// Switches the drone into crash behaviour.
    /// The drone waits for all senders of it's own receiver to be removed,
    /// and then breaks.
    pub(super) fn crash_behaviour(&mut self) {
        while let Ok(packet) = self.packet_recv.recv() {
            self.handle_packet_crashed(packet);
        }
    }

    /// In crash behaviour:
    /// - MsgFragments: Return error in routing
    /// - Nack/Ack/FloodResponse: Are handled as usual
    /// - FloodRequests: ignored :(
    pub(super) fn handle_packet_crashed(&mut self, mut packet: Packet) {
        match packet.pack_type {
            PacketType::MsgFragment(_) => {
                header::increment_index(&mut packet.routing_header);
                self.handle_send_error(packet, self.id);
            }
            PacketType::Ack(_) | PacketType::Nack(_) | PacketType::FloodResponse(_) => {
                let res = self.check_packet(packet);
                if let Some(pack_ready) = self.handle_check_result(res) {
                    let sender_res = self.get_send_info(pack_ready);
                    if sender_res.is_none() {
                        return;
                    }

                    let (pack_ready, id, sender) = sender_res.unwrap();
                    self.log_action(pack_ready.clone(), false);
                    self.forward(pack_ready, id, sender);
                }
            }
            PacketType::FloodRequest(_) => (),
        }
    }
}
