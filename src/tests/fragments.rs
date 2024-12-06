#[cfg(test)]
mod tests {

    use crate::RustafarianDrone;
    use crossbeam_channel::{select_biased, unbounded, Receiver, Sender};
    use rand::*;
    use std::collections::{HashMap, HashSet};
    use std::ops::Index;
    use std::{fs, thread};
    use wg_2024::config::Config;
    use wg_2024::controller::{DroneCommand, DroneEvent};
    use wg_2024::drone::Drone;
    use wg_2024::network::{NodeId, SourceRoutingHeader};
    use wg_2024::packet::PacketType::Nack;
    use wg_2024::packet::{Ack, FloodRequest, FloodResponse, NackType, NodeType};
    use wg_2024::packet::{Packet, PacketType};
    use wg_2024::tests;

    #[test]
    fn run_fragment_drop() {
        tests::generic_fragment_drop::<RustafarianDrone>();
    }

    #[test]
    fn run_fragment_forward() {
        tests::generic_fragment_forward::<RustafarianDrone>();
    }

    #[test]
    fn run_chain_fragment_ack() {
        tests::generic_chain_fragment_ack::<RustafarianDrone>();
    }

    #[test]
    fn run_chain_fragment_drop() {
        tests::generic_chain_fragment_drop::<RustafarianDrone>();
    }

    #[test]
    fn run_complex_topology() {
        // Define a more complex topology with multiple drones and paths
        let (c_send, c_recv) = unbounded();
        let (s_send, s_recv) = unbounded();
        let (d1_send, d1_recv) = unbounded();
        let (d2_send, d2_recv) = unbounded();
        let (d3_send, d3_recv) = unbounded();
        let (_d_command_send, d_command_recv) = unbounded();

        let neighbours1 = HashMap::from([
            (12, d2_send.clone()),
            (13, d3_send.clone()),
            (1, c_send.clone()),
        ]);
        let neighbours2 = HashMap::from([
            (11, d1_send.clone()),
            (13, d3_send.clone()),
            (21, s_send.clone()),
        ]);
        let neighbours3 = HashMap::from([(11, d1_send.clone()), (12, d2_send.clone())]);

        let mut drone1 = RustafarianDrone::new(
            11,
            unbounded().0,
            d_command_recv.clone(),
            d1_recv.clone(),
            neighbours1,
            0.0,
        );
        let mut drone2 = RustafarianDrone::new(
            12,
            unbounded().0,
            d_command_recv.clone(),
            d2_recv.clone(),
            neighbours2,
            0.0,
        );
        let mut drone3 = RustafarianDrone::new(
            13,
            unbounded().0,
            d_command_recv.clone(),
            d3_recv.clone(),
            neighbours3,
            0.0,
        );

        thread::spawn(move || {
            drone1.run();
        });
        thread::spawn(move || {
            drone2.run();
        });
        thread::spawn(move || {
            drone3.run();
        });

        let mut msg = create_sample_packet();
        d1_send.send(msg.clone()).unwrap();
        msg.routing_header.hop_index = 3;

        // // First message: ACK
        // c_recv.recv().unwrap();
        // // Second message: ACK
        // c_recv.recv().unwrap();

        assert_eq!(s_recv.recv().unwrap(), msg);
        // assert_eq!(c_recv.recv().unwrap(), msg);
    }

    #[test]
    fn run_edge_cases() {
        // Test edge cases such as packet drop rates, invalid routing headers, etc.
        let (c_send, c_recv) = unbounded();
        let (d_send, d_recv) = unbounded();
        let (_d_command_send, d_command_recv) = unbounded();

        let neighbours = HashMap::from([(12, d_send.clone()), (1, c_send.clone())]);
        let mut drone = RustafarianDrone::new(
            11,
            unbounded().0,
            d_command_recv,
            d_recv.clone(),
            neighbours,
            1.0,
        );

        thread::spawn(move || {
            drone.run();
        });

        let msg = create_sample_packet();
        d_send.send(msg.clone()).unwrap();

        let dropped = wg_2024::packet::Nack {
            fragment_index: 1,
            nack_type: NackType::Dropped,
        };
        let srh = SourceRoutingHeader {
            hop_index: 1,
            hops: vec![11, 1],
        };
        let nack_packet = Packet {
            pack_type: PacketType::Nack(dropped),
            routing_header: srh,
            session_id: 1,
        };

        assert_eq!(c_recv.recv().unwrap(), nack_packet);
    }

    fn create_sample_packet() -> Packet {
        Packet {
            pack_type: PacketType::MsgFragment(wg_2024::packet::Fragment {
                fragment_index: 1,
                total_n_fragments: 1,
                length: 128,
                data: [2; 128],
            }),
            routing_header: SourceRoutingHeader {
                hop_index: 1,
                hops: vec![1, 11, 12, 21],
            },
            session_id: 1,
        }
    }
}
