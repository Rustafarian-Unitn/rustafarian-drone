#[cfg(test)]
mod simcontroller_tests {
    use crate::RustafarianDrone;
    use crate::SourceRoutingHeader;
    use crossbeam_channel::{unbounded, Receiver, Sender};
    use wg_2024::packet::Ack;
    use wg_2024::packet::Fragment;
    use std::collections::HashMap;
    use std::thread;
    use wg_2024::drone::Drone;
    use wg_2024::packet::{
        FloodRequest, FloodResponse, Nack, NackType, NodeType, Packet, PacketType,
    };
    use wg_2024::tests;

    /// Tests whether a crashed node forwards an ACK
    #[test]
    fn run_crash_ack() {
        let (c_send, c_recv) = unbounded();
        let (s_send, s_recv) = unbounded();
        let (d1_send, d1_recv) = unbounded();
        let (d2_send, d2_recv) = unbounded();
        let (_d_command_send, d_command_recv) = unbounded();

        let neighbours1 = HashMap::from([(12, d2_send.clone()), (1, c_send.clone())]);
        let neighbours2 = HashMap::from([(11, d1_send.clone()), (21, s_send.clone())]);
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
            unbounded().1,
            d2_recv.clone(),
            neighbours2,
            0.0,
        );

        thread::spawn(move || {
            drone1.run();
        });
        thread::spawn(move || {
            drone2.run();
        });
        _d_command_send
            .send(wg_2024::controller::DroneCommand::Crash {})
            .unwrap();

        let mut msg = Packet {
            routing_header: SourceRoutingHeader {
                hop_index: 1,
                hops: [1, 11, 12].to_vec(),
            },
            pack_type: PacketType::Ack(Ack { fragment_index: 0 }),
            session_id: 0,
        };

        d1_send.send(msg.clone());

        msg.routing_header.hop_index += 1;

        assert_eq!(
            d2_recv.recv().unwrap(),
            msg,
            "The two messages are not the same!"
        );
    }

    /// Tests whether a crashed node returns a NACK when trying to send a packet
    #[test]
    fn run_crash_msg() {
        let (c_send, c_recv) = unbounded();
        let (s_send, s_recv) = unbounded::<Packet>();
        let (d1_send, d1_recv) = unbounded();
        let (_d_command_send, d_command_recv) = unbounded();

        let neighbours1 = HashMap::from([(1, c_send.clone())]);

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

        _d_command_send
            .send(wg_2024::controller::DroneCommand::Crash {})
            .unwrap();

        let mut msg = Packet {
            routing_header: SourceRoutingHeader {
                hop_index: 1,
                hops: [1, 11, 12].to_vec(),
            },
            pack_type: PacketType::MsgFragment(Fragment {
                fragment_index: 0,
                total_n_fragments: 1,
                length: 3,
                data: [2; 128],
            }),
            session_id: 0,
        };

        d1_send.send(msg.clone());

        let nack = PacketType::Nack(Nack {
            fragment_index: 0,
            nack_type: NackType::ErrorInRouting(11),
        });

        assert_eq!(
            c_recv.recv().unwrap().pack_type,
            nack,
            "The two messages are not the same!"
        );
    }
}
