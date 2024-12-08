#[cfg(test)]
mod flood_tests {
    use crate::RustafarianDrone;
    use crate::SourceRoutingHeader;
    use crossbeam_channel::{unbounded, Receiver, Sender};
    use std::collections::HashMap;
    use std::thread;
    use wg_2024::drone::Drone;
    use wg_2024::packet::{
        FloodRequest, FloodResponse, Nack, NackType, NodeType, Packet, PacketType,
    };
    use wg_2024::tests;

    /// Test that a flood response is forwarded correctly
    #[test]
    fn run_flooding_response() {
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

        let mut flood_response = Packet {
            session_id: 0,
            routing_header: SourceRoutingHeader {
                hop_index: 1,
                hops: [21, 11, 13, 12, 21].to_vec(),
            },
            pack_type: PacketType::FloodResponse(FloodResponse {
                flood_id: 0,
                path_trace: [
                    (11, NodeType::Drone),
                    (12, NodeType::Drone),
                    (13, NodeType::Drone),
                ]
                .to_vec(),
            }),
        };

        d1_send.send(flood_response.clone()).unwrap();

        flood_response.routing_header.hop_index += 3;

        assert_eq!(
            s_recv.recv().unwrap(),
            flood_response,
            "Response not received"
        );
    }

    #[test]
    fn run_flooding_req() {
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
                assert_eq!(
                    _response.flood_id, flood_request.flood_id,
                    "The response ID is different from the flood request id"
                );
            }
            _ => assert!(false, "The packet received was not a response?!"),
        }

        match s_recv.recv().unwrap().pack_type {
            PacketType::FloodRequest(_response) => {
                assert_eq!(
                    _response.flood_id, flood_request.flood_id,
                    "The response ID is different from the flood request id"
                );
            }
            _ => assert!(false, "The packet received was not a response?!"),
        }
    }
}
