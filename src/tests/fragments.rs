use wg_2024::tests;

use crate::RustafarianDrone;

#[test]
fn run_all_tests() {
    // println!("Running drop test");
    // tests::generic_fragment_drop::<RustafarianDrone>();
    // ... call all other tests similarly
}

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