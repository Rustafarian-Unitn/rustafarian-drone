#![allow(unused)]
use crossbeam_channel::{select_biased, unbounded, Receiver, Sender};
use std::collections::{HashMap, HashSet};
use std::{fs, thread};
use std::ops::Index;
use wg_2024::config::Config;
use wg_2024::controller::{DroneCommand, DroneEvent};
use wg_2024::drone::Drone;
use wg_2024::network::{NodeId, SourceRoutingHeader};
use wg_2024::packet::{FloodRequest, FloodResponse, NackType, NodeType};
use wg_2024::packet::{Packet, PacketType};
use rand::*;
use wg_2024::packet::PacketType::Nack;


pub struct RustafarianDrone {
    id: NodeId,
    controller_send: Sender<DroneEvent>,
    controller_recv: Receiver<DroneCommand>,
    packet_recv: Receiver<Packet>,
    pdr: f32,
    neighbors: HashMap<NodeId, Sender<Packet>>, // Packet send in Drone interface
    flood_requests: HashSet<u64>, // Contains: O(1) in average
    crashed: bool,
}

impl Drone for RustafarianDrone {
    fn new(
        id: NodeId,
        controller_send: Sender<DroneEvent>,
        controller_recv: Receiver<DroneCommand>,
        packet_recv: Receiver<Packet>,
        packet_send: HashMap<NodeId, Sender<Packet>>,
        pdr: f32,
    ) -> Self {
        Self {
            id,
            controller_send,
            controller_recv,
            packet_recv,
            neighbors: packet_send,
            pdr,
            flood_requests: HashSet::new(),
            crashed: false
        }
    }

    fn run(&mut self) {
        loop {
            select_biased! {
                recv(self.controller_recv) -> command => {
                    if let Ok(command) = command {
                        if let DroneCommand::Crash = command {
                            println!("drone {} crashed", self.id);
                            break;
                        }
                        self.handle_command(command);
                    }
                }
                recv(self.packet_recv) -> packet => {
                    if let Ok(packet) = packet {
                        self.handle_packet(packet);
                    }
                },
            }
        }
    }
}

impl RustafarianDrone {
    fn handle_packet(&mut self, packet: Packet) {
        match packet.clone().pack_type {
            PacketType::Nack(_nack) => self.forward_packet(packet, true),
            PacketType::Ack(_ack) => self.forward_packet(packet, true),
            PacketType::MsgFragment(_fragment) => self.forward_packet(packet, false),
            PacketType::FloodRequest(_flood_request) => {
                println!("Flood request arrived at {:?}", self.id);
                self.handle_flood_req(_flood_request, packet.session_id, packet.routing_header);
            }
            PacketType::FloodResponse(_flood_response) => {
                println!("Flood response arrived at {:?}", self.id);
            }
        }
    }

    fn handle_command(&mut self, command: DroneCommand) {
        match command {
            DroneCommand::AddSender(node_id, sender) => self.add_neighbor(node_id, sender),
            DroneCommand::SetPacketDropRate(pdr) => self.set_packet_drop_rate(pdr),
            DroneCommand::RemoveSender(node_id) => self.remove_sender(node_id),
            DroneCommand::Crash => self.make_crash(),
        }
    }

    fn add_neighbor(&mut self, node_id: u8, neighbor: Sender<Packet>) {
        self.neighbors.insert(node_id, neighbor);
    }

    fn remove_sender(&mut self, node_id: u8) {

    }
    fn make_crash(&mut self) {
        self.crashed = true;
    }

    fn set_packet_drop_rate(&mut self, pdr: f32) {
        self.pdr = pdr;
    }

    fn should_drop(&self) -> bool {
        return rand::thread_rng().gen_range(0.0..100.0) < self.pdr;
    }

    fn forward_packet(&mut self, packet: Packet, skip_pdr_check: bool) {
        // Step 1: check I'm the intended receiver
        let curr_hop = packet.routing_header.hops[packet.routing_header.hop_index];
        if self.id != curr_hop {
            // Error, I'm not the one who's supposed to receive this
            self.send_nack(packet, NackType::UnexpectedRecipient(self.id));
            return;
        }

        let mut new_packet = packet.clone();
        // Step 2: increase the hop index
        new_packet.routing_header.hop_index += 1;
        let next_hop_index = new_packet.routing_header.hop_index;
        
        // Step 3: check I'm not the last hop
        if next_hop_index >= packet.routing_header.hops.len() {
            // Error, I'm the last hop!
            self.send_nack(packet, NackType::DestinationIsDrone);
            return;
        }
        
        self.send_packet(new_packet, skip_pdr_check);
    }

    fn send_packet(&mut self, packet: Packet, skip_pdr_check: bool) -> bool {

        let mut result = false;

        let next_hop_index = packet.routing_header.hop_index;
        // Check I have the next hop as neighbor
        let next_hop = packet.routing_header.hops[next_hop_index];

        match self.neighbors.get(&next_hop) {
            Some(channel) => {

                // Check if packet can be dropped, if so check the PDR
                if !skip_pdr_check && self.should_drop() {

                    self.send_nack(packet.clone(), NackType::Dropped);
                    // Notify controller that a packet has been dropped
                    self.controller_send.send(DroneEvent::PacketDropped(packet));

                    return false;
                }

                match channel.send(packet.clone()) {
                    Ok(()) => {
                        // Notify controller that a packet has been correctly sent
                        self.controller_send.send(DroneEvent::PacketSent(packet));
                        result = true;
                    },
                    Err(error) => {
                        // Should never reach this error, SC should prevent it
                        println!("Error while sending packet on closed channel");
                        self.send_nack(packet, NackType::ErrorInRouting(next_hop))
                    }
                }
            },
            None => {
                self.send_nack(packet, NackType::ErrorInRouting(next_hop))
            }
        }
        result
    }

    fn send_nack(&mut self, packet: Packet, nack_type: NackType) {

        // Get index for the current node
        let self_index = packet
            .routing_header
            .hops
            .iter()
            .position(|id| id == &self.id).unwrap();


        let mut route: Vec<u8> = packet.clone()
            .routing_header
            .hops
            .split_at(self_index)
            .0
            .clone()
            .into_iter()
            .collect();
        route.reverse();

        let routing_header = SourceRoutingHeader{
            hop_index : 1,
            hops : route
        };

        let nack = wg_2024::packet::Nack{
            fragment_index: 0,
            nack_type
        };

        let packet = Packet{
            pack_type: Nack(nack),
            routing_header,
            session_id: packet.session_id
        };

        if !self.send_packet(packet.clone(), true) {
            // Nack can't be forwarded, send it to SC
            self.controller_send.send(DroneEvent::ControllerShortcut(packet));
        }
    }

    /**
     * When a flood request packet is received:
     * 1. The drone adds itself to the path_trace
     * 2.If the ID is in the memory: create and send a FloodResponse
     * 3. Otherwise:
     *  3.1 if has neighbors forwards the packet to its neighbors
     *  3.2 if no neighbors send it to node from which it received it
     */
    pub fn handle_flood_req(
        &mut self,
        packet: FloodRequest,
        session_id: u64,
        routing_header: SourceRoutingHeader,
    ) {
        // If we have the ID in memory, and the path trace contains our ID
        // Request already handled, prepare response
        /** && packet.clone().path_trace.iter().any(|node| node.0 == self.id) */
        if self.flood_requests.contains(&packet.flood_id) {
            let mut flood_request_clone = packet.clone();
            // Get the ID of the drone that sent the request
            let sender_id = flood_request_clone.path_trace.last().unwrap().0;
            // Add myself to the path trace
            flood_request_clone
                .path_trace
                .push((self.id, NodeType::Drone));
            let mut route: Vec<u8> = flood_request_clone
                .path_trace
                .clone()
                .into_iter()
                .map(|node| node.0)
                .collect();
            route.reverse();

            let response = FloodResponse {
                flood_id: packet.flood_id,
                path_trace: flood_request_clone.path_trace,
            };

            // Create the packet with the route provided in the path trace
            let new_packet = Packet {
                pack_type: PacketType::FloodResponse(response),
                session_id,
                routing_header: SourceRoutingHeader {
                    hop_index: 1,
                    hops: route,
                },
            };

            match self.neighbors.get(&sender_id) {
                Some(channel) => match channel.send(new_packet) {
                    Ok(()) => {},
                    Err(error) => println!("Couldn't send response, as the neighbor has crashed")
                },
                _ => {}
            }
        } else {
            // Send to neighbors
            let mut new_packet = packet.clone();
            // Save the last node's ID, we don't want to send the request to it
            let last_node = (new_packet.path_trace.last().unwrap().0);
            // Add our ID to the trace
            new_packet.path_trace.push((self.id, NodeType::Drone));

            // Send to all neighbors
            for neighbor in &self.neighbors {
                let neighbor_id = neighbor.0;
                let neighbor_channel = neighbor.1;
                if neighbor_id == &last_node {
                    continue;
                }

                match neighbor_channel.send(Packet {
                    pack_type: PacketType::FloodRequest(new_packet.clone()),
                    routing_header: routing_header.clone(),
                    session_id,
                }) {
                    Ok(()) => {}
                    Err(error) => println!("Couldn't send response, as the neighbor has crashed")
                }
            }
        }
    }
}
