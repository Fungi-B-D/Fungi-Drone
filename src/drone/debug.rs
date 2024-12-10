use wg_2024::packet::Packet;

use super::FungiDrone;

impl FungiDrone {
    pub(super) fn debug(&self, debug_message: &str, debug_packet: Option<Packet>) {
        if self.debug_print {
            println!("[{}]: {debug_message}", self.id);
        }
        if self.debug_shortcut {
            self.send_controller(wg_2024::controller::DroneEvent::ControllerShortcut(
                debug_packet.unwrap(),
            ));
        }
    }

    pub fn set_debug_print(&mut self, state: bool) {
        self.debug_print = state;
    }

    pub fn set_debug_shortcut(&mut self, state: bool) {
        self.debug_shortcut = state;
    }
}
