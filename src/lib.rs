#![allow(unused)]
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
mod tests;

pub struct RustafarianDrone {
    id: NodeId,                                 // The ID of the drone, u8
    controller_send: Sender<DroneEvent>,        // Send messages to the Sim Controller
    controller_recv: Receiver<DroneCommand>,    // Receive messages from the Sim Controller
    packet_recv: Receiver<Packet>,              // Receive messages from other drones
    pdr: f32,                                   // Packet Drop Rate
    neighbors: HashMap<NodeId, Sender<Packet>>, // Map containing the neighbors of the current drone. The key is the ID of the neighbor, the value is the channel
    flood_requests: HashSet<u64>,               // Contains: O(1) in average
    crashed: bool,                              // Whether the drone is crashed
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
            crashed: false,
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
    /**
     * Handle packets that arrive from other drones.
     */
    fn handle_packet(&mut self, packet: Packet) {
        // Packets are cloned before the handle otherwise they get consumed by the arms execution
        let pack_type = &packet.pack_type;
        match pack_type {
            PacketType::Nack(_nack) => self.forward_packet(&packet, true, 0),
            PacketType::Ack(_ack) => self.forward_packet(&packet, true, 0),
            PacketType::MsgFragment(_fragment) => {
                self.forward_packet(&packet, false, _fragment.fragment_index)
            }
            PacketType::FloodRequest(_flood_request) => {
                println!("Flood request arrived at {:?}", self.id);
                self.handle_flood_req(_flood_request, packet.session_id, packet.routing_header);
            }
            PacketType::FloodResponse(_flood_response) => {
                println!("Flood response arrived at {:?}", self.id);
            }
        }
    }

    /**
     * Handle commands from the Simulation Controller
     */
    fn handle_command(&mut self, command: DroneCommand) {
        match command {
            DroneCommand::AddSender(node_id, sender) => self.add_neighbor(node_id, sender),
            DroneCommand::SetPacketDropRate(pdr) => self.set_packet_drop_rate(pdr),
            DroneCommand::RemoveSender(node_id) => self.remove_sender(node_id),
            DroneCommand::Crash => self.make_crash(),
        }
    }

    /**
     * Add a neighbor to the list, can only be called by the sim controller.
     * node_id: The ID for the new node;
     * neighbor: The sender channel for the new node.
     */
    fn add_neighbor(&mut self, node_id: u8, neighbor: Sender<Packet>) {
        self.neighbors.insert(node_id, neighbor);
    }

    /**
     * Remove a node from the neighbors using the ID. Can only be called by the sim controller.
     */
    fn remove_sender(&mut self, node_id: u8) {
        self.neighbors.remove(&node_id);
    }

    /**
     * Set the status of the drone as crashed. Can only be called by the sim controller
     */
    fn make_crash(&mut self) {
        self.crashed = true;
    }

    /**
     * Change the packet drop rate. Can only be called by the sim controller.
     */
    fn set_packet_drop_rate(&mut self, pdr: f32) {
        self.pdr = pdr;
    }

    /**
     * Check whether the packet should be dropped, using a random number and the Packet Drop Rate.
     * Returns true if the packet should be dropped, false otherwise.
     */
    fn should_drop(&self) -> bool {
        return rand::thread_rng().gen_range(0.0..1.0) < self.pdr;
    }

    /**
     * Forwards a packet to the next node, doing checks such as:
     * 1. The current drone is the intended receiver
     * 2. Check that the drone is not the last hop
     * packet: the packet to forward;
     * skip_pdr_check: If it should skip the check for the Packet Drop Rate before sending. True for ACKs, NACKs, Flood messages.
     */
    fn forward_packet(&mut self, packet: &Packet, skip_pdr_check: bool, fragment_index: u64) {
        // Step 1: check I'm the intended receiver
        let curr_hop = packet.routing_header.hops[packet.routing_header.hop_index];
        if self.id != curr_hop {
            // Error, I'm not the one who's supposed to receive this
            self.send_nack_fragment(
                packet,
                NackType::UnexpectedRecipient(self.id),
                fragment_index,
            );
            return;
        }

        let mut new_packet = packet.clone();
        // Step 2: increase the hop index
        new_packet.routing_header.hop_index += 1;

        let next_hop_index = packet.routing_header.hop_index;

        // Step 3: check I'm not the last hop
        if next_hop_index >= packet.routing_header.hops.len() {
            // Error, I'm the last hop!
            self.send_nack_fragment(packet, NackType::DestinationIsDrone, fragment_index);
            return;
        }

        // Skip_pdr_check: true only for ACK and NACK, so in those two cases we don't send the ACK back to the sender.
        if self.send_packet(&new_packet, skip_pdr_check, fragment_index) && !skip_pdr_check {
            // TODO: send ACK to previous hop in the list.
            let ack = Ack { fragment_index };

            self.send_back(packet, PacketType::Ack(ack));
        }
    }

    /**
     * Send a packet to the target's channel. The target is the next node in the routing header.
     * Packet: the packet to send.
     * Skip_pdr_check: whether the check for the drop of the packet should be skipped. Only the fragments can be lost,
     * so it's true for ACKs, NACKs, flooding messages
     * Return true if the packet was sent successfully, false otherwise.
     */
    fn send_packet(&mut self, packet: &Packet, skip_pdr_check: bool, fragment_index: u64) -> bool {
        let mut result = false;

        let next_hop_index = packet.routing_header.hop_index;

        // Check if the next_hop_index is valid
        if next_hop_index >= packet.routing_header.hops.len() {
            println!(
                "Error: next_hop_index ({}) >= packet.routing_header.hops.len() ({})",
                next_hop_index, packet.routing_header.hop_index
            );
            return false;
        }

        // Check I have the next hop as neighbor
        let next_hop = packet.routing_header.hops[next_hop_index];

        match self.neighbors.get(&next_hop) {
            Some(channel) => {
                // Check if packet can be dropped, if so check the PDR
                if !skip_pdr_check && self.should_drop() {
                    let nack = wg_2024::packet::Nack {
                        fragment_index,
                        nack_type: NackType::Dropped,
                    };

                    self.send_back(packet, PacketType::Nack(nack));
                    // Notify controller that a packet has been dropped
                    self.controller_send
                        .send(DroneEvent::PacketDropped(packet.clone()));

                    return false;
                }

                match channel.send(packet.clone()) {
                    Ok(()) => {
                        // Notify controller that a packet has been correctly sent
                        self.controller_send
                            .send(DroneEvent::PacketSent(packet.clone()));
                        result = true;
                        // println!("Sent");
                    }
                    Err(error) => {
                        // Should never reach this error, SC should prevent it
                        println!("Error while sending packet on closed channel");
                        let nack = wg_2024::packet::Nack {
                            fragment_index,
                            nack_type: NackType::ErrorInRouting(next_hop),
                        };

                        self.send_back(packet, PacketType::Nack(nack));
                    }
                }
            }
            None => {
                // Next hop is not my neighbour
                let nack = wg_2024::packet::Nack {
                    fragment_index,
                    nack_type: NackType::ErrorInRouting(next_hop),
                };

                self.send_back(packet, PacketType::Nack(nack));
            }
        }

        result
    }

    /**
     * Send a NACK packet to the previous node.
     * The target is taken by reversing the routing header, starting from the current hop.
     * Packet: the packet that couldn't be sent;
     * nack_type: what cause the packet to be lost (Drop, Error in routing...)
     */
    fn send_nack_fragment(&mut self, packet: &Packet, nack_type: NackType, fragment_index: u64) {
        // Get index for the current node

        let nack = wg_2024::packet::Nack {
            fragment_index,
            nack_type,
        };

        if !self.send_back(packet, PacketType::Nack(nack)) {
            // Nack can't be forwarded, send it to SC
            self.controller_send
                .send(DroneEvent::ControllerShortcut(packet.clone()));
        }
    }

    /**
     * Calls send_nack_fragment(packet, nack_type, 0)
     * Use in case the original packet was not a fragment
     */
    // fn send_nack(&mut self, packet: Packet, nack_type: NackType) {
    //     self.send_nack_fragment(packet, nack_type, 0);
    // }

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
        packet: &FloodRequest,
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
                    Ok(()) => {}
                    Err(error) => println!("Couldn't send response, as the neighbor has crashed"),
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

                // Avoid sending to the node that send the request to us
                if neighbor_id == &last_node {
                    continue;
                }

                match neighbor_channel.send(Packet {
                    pack_type: PacketType::FloodRequest(new_packet.clone()),
                    routing_header: routing_header.clone(),
                    session_id,
                }) {
                    Ok(()) => {}
                    Err(error) => println!("Couldn't send response, as the neighbor has crashed"),
                }
            }
        }
    }

    fn send_back(&mut self, acked_packet: &Packet, acknowledgment: PacketType) -> bool {
        let self_index = acked_packet
            .routing_header
            .hops
            .iter()
            .position(|id| id == &self.id)
            .unwrap();

        let mut route: Vec<u8> = acked_packet.clone().routing_header.hops;
        route.truncate(self_index + 1);
        route.reverse();

        let routing_header = SourceRoutingHeader {
            hop_index: 1,
            hops: route,
        };

        let new_packet = Packet {
            pack_type: acknowledgment,
            session_id: acked_packet.session_id,
            routing_header,
        };

        self.send_packet(&new_packet, true, 0)
    }
}

fn log(msg: String) {
    println!("{}", msg);
}
