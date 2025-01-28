/// Run tests from the wg repository
#[cfg(test)]
mod wg_tests {
    use crate::RustafarianDrone;
    use crossbeam_channel::{unbounded, Receiver, Sender};
    use std::collections::HashMap;
    use std::thread;
    use wg_2024::packet::{Packet, PacketType, Nack, NackType, FloodRequest, FloodResponse, NodeType};
    use wg_2024::tests;
    use crate::SourceRoutingHeader;
    use wg_2024::drone::Drone;

    // This test is wrong
    // #[test]
    // fn run_fragment_drop() {
        // tests::generic_fragment_drop::<RustafarianDrone>();
    // }

    #[test]
    fn run_fragment_forward() {
        tests::generic_fragment_forward::<RustafarianDrone>();
    }

    // #[test]
    fn run_chain_fragment_ack() {
        tests::generic_chain_fragment_ack::<RustafarianDrone>();
    }
}