use wg_2024::tests;

use crate::RustafarianDrone;

#[test]
fn run_all_tests() {
    println!("Running drop test");
    tests::generic_fragment_drop::<RustafarianDrone>();
    // ... call all other tests similarly
}
