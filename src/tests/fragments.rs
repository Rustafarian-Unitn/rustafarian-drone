/// Run tests on message fragments
#[cfg(test)]
mod fragment_tests {
    use crate::RustafarianDrone;
    use crossbeam_channel::{unbounded, Receiver, Sender};
    use std::collections::HashMap;
    use std::thread;
    use wg_2024::packet::{Packet, PacketType, Nack, NackType, FloodRequest, FloodResponse, NodeType};
    use wg_2024::tests;
    use crate::SourceRoutingHeader;
    use wg_2024::drone::Drone;

    /// Check that a message is forwarded correctly
    #[test]
    fn run_complex_topology() {
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

        assert_eq!(s_recv.recv().unwrap(), msg);
    }

    /// Tests that a fragment is dropped correctly
    #[test]
    fn run_edge_cases() {
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
            1.0,
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

        let msg = create_sample_packet();
        d1_send.send(msg.clone()).unwrap();

        let dropped = Nack {
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

    /// Check that the NACK is forwarded correctly.
    #[test]
    fn run_nack() {
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

        let dropped = Nack {
            fragment_index: 1,
            nack_type: NackType::Dropped,
        };
        let srh = SourceRoutingHeader {
            hop_index: 1,
            hops: vec![1, 11, 13, 12, 21],
        };
        let mut nack_packet = Packet {
            pack_type: PacketType::Nack(dropped),
            routing_header: srh,
            session_id: 1,
        };

        d1_send.send(nack_packet.clone()).unwrap();

        nack_packet.routing_header.hop_index += 3;

        assert_eq!(s_recv.recv().unwrap(), nack_packet);
    }

    /// Check that if the drone is not in the neighbors, the result is a NACK of type ErrorInRouting
    #[test]
    fn run_wrong_neighbor() {
        let (c_send, c_recv) = unbounded();
        let (s_send, s_recv) = unbounded::<Packet>();
        let (d1_send, d1_recv) = unbounded();
        let (_d_command_send, d_command_recv) = unbounded();

        let neighbours1 = HashMap::from([
            (1, c_send.clone()),
        ]);

        let mut drone1 = RustafarianDrone::new(
            11,
            unbounded().0,
            d_command_recv.clone(),
            d1_recv.clone(),
            neighbours1,
            0.0,
        );

        thread::spawn(move || {
            drone1.run();
        });

        let mut msg = create_sample_packet();
        d1_send.send(msg).unwrap();

        let expected_nack = Packet {
            session_id: 1,
            routing_header: SourceRoutingHeader {
                hop_index: 1,
                hops: [11, 1].to_vec()
            },
            pack_type: PacketType::Nack(Nack {
                fragment_index: 1,
                nack_type: NackType::ErrorInRouting(12)
            })
        };

        assert_eq!(c_recv.recv().unwrap(), expected_nack, "The message received should be an error in routing!");
    }
}