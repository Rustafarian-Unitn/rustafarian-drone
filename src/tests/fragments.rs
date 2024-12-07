#[cfg(test)]
mod tests {
    use crate::RustafarianDrone;
    use crossbeam_channel::{unbounded, Receiver, Sender};
    use std::collections::HashMap;
    use std::thread;
    use wg_2024::packet::{Packet, PacketType, Nack, NackType, FloodRequest, FloodResponse, NodeType};
    use wg_2024::tests;
    use crate::SourceRoutingHeader;
    use wg_2024::drone::Drone;


    struct Topology {
        c_send: Sender<Packet>,
        c_recv: Receiver<Packet>,
        s_send: Sender<Packet>,
        s_recv: Receiver<Packet>,
        d1_send: Sender<Packet>,
        d1_recv: Receiver<Packet>,
        d2_send: Sender<Packet>,
        d2_recv: Receiver<Packet>,
        d3_send: Sender<Packet>,
        d3_recv: Receiver<Packet>,
    }

    #[test]
    fn run_fragment_drop() {
        tests::generic_fragment_drop::<RustafarianDrone>();
    }

    #[test]
    fn run_fragment_forward() {
        tests::generic_fragment_forward::<RustafarianDrone>();
    }

    // #[test]
    // fn run_chain_fragment_ack() {
    //     tests::generic_chain_fragment_ack::<RustafarianDrone>();
    // }

    // #[test]
    // fn run_chain_fragment_drop() {
    //     tests::generic_chain_fragment_drop::<RustafarianDrone>();
    // }

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

    #[test]
    fn run_flooding() {
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

        let flood_request = FloodRequest {
            flood_id: 1,
            initiator_id: 1,
            path_trace: vec![(1, NodeType::Client)],
        };

        let packet = Packet {
            pack_type: PacketType::FloodRequest(flood_request.clone()),
            routing_header: SourceRoutingHeader {
                hop_index: 0,
                hops: vec![],
            },
            session_id: 1,
        };

        let mut flood_request_d2 = flood_request.clone();
        let mut flood_request_d3 = flood_request.clone();
        flood_request_d2.path_trace.push((11, NodeType::Drone));
        flood_request_d3.path_trace.push((13, NodeType::Drone));

        let packet_received_d2 = Packet {
            pack_type: PacketType::FloodRequest(flood_request_d2),
            routing_header: SourceRoutingHeader {
                hop_index: 0,
                hops: vec![],
            },
            session_id: 1,
        };

        let packet_received_d3 = Packet {
            pack_type: PacketType::FloodRequest(flood_request_d3),
            routing_header: SourceRoutingHeader {
                hop_index: 0,
                hops: vec![],
            },
            session_id: 1,
        };

        d1_send.send(packet.clone()).unwrap();


        let mut path_trace = vec![
            (1, NodeType::Client),
            (11, NodeType::Drone),
            (12, NodeType::Drone),
            (21, NodeType::Server),
        ];
        path_trace.reverse();

        let flood_response = Packet {
            pack_type: PacketType::FloodResponse(FloodResponse {
                flood_id: 1,
                path_trace,
            }),
            routing_header: SourceRoutingHeader {
                hop_index: 0,
                hops: vec![],
            },
            session_id: 1,
        };

        // d2_send.send(flood_response.clone()).unwrap();
        match c_recv.recv().unwrap().pack_type {
            PacketType::FloodResponse(_response) => {
                assert_eq!(_response.flood_id, flood_request.flood_id, "The response ID is different from the flood request id");
            }
            _ => assert!(false, "The packet received was not a response?!")
        }

        match s_recv.recv().unwrap().pack_type {
            PacketType::FloodRequest(_response) => {
                assert_eq!(_response.flood_id, flood_request.flood_id, "The response ID is different from the flood request id");
            }
            _ => assert!(false, "The packet received was not a response?!")
        }
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