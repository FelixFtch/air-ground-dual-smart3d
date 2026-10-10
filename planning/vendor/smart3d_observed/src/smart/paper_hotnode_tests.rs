//! SMART-3D II-C1/C2/C3 regression fixtures; no random route or robot movement.
use super::*;
use crate::obstacles::AnalyticValidityChecker;
use crate::rrt::{LinearNearestNeighbors, UniformDistribution};

type S = RealVectorState<f32, 2>;
type P = SMART<f32, 2, LinearNearestNeighbors<f32, 2>>;
type Sphere = StaticSphericalObstacle<f32, 2>;

fn planner(rows: &[([f32; 2], Option<usize>, i32)], actual: [f32; 2]) -> P {
    let mut nn = LinearNearestNeighbors::new();
    let nodes: Vec<_> = rows.iter().enumerate().map(|(id, (xy, parent, _))| {
        nn.add(S::new(*xy), id); rrt_star::Node::new(S::new(*xy), *parent, 0.)
    }).collect();
    let mut p = SMART::new_from_initial_rrt_star_tree(&nodes, nn, S::new(actual), S::new(rows[0].0),
        0.01, Box::new(AnalyticValidityChecker::<f32, 2, Sphere>::new(vec![])),
        Box::new(UniformDistribution::new([(-20., 20.), (-20., 20.)])),
        9., 1., 1.5, 10., 1.7, 1.7, false, 0.01);
    p.initial_planning_phase = false;
    for (id, (_, _, tree_id)) in rows.iter().enumerate() {
        p.nodes[id].tree_id = *tree_id;
        if *tree_id != 0 { p.nodes[id].cumulative_cost = f32::INFINITY; }
    }
    p.tree_id_counter = rows.iter().map(|r| r.2).max().unwrap().max(0);
    assert!(p.debug_check_finalized_forest()); p
}

fn two_routes() -> P {
    planner(&[
        ([0., 8.], None, 0),
        ([-6., 8.], Some(0), 0), ([-6., 0.], Some(1), 0), ([-1.4, 0.], Some(2), 0),
        ([1.4, 8.], Some(0), 0), ([1.4, 0.], Some(4), 0),
        ([-0.5, 0.], Some(7), 1), ([0.5, 0.], None, 1),
    ], [0., 0.])
}

#[test]
fn two_hot_nodes_rank_larger_inverse_of_full_route_cost_first() {
    let mut p = two_routes();
    let slow = p.utility(p.node(6), p.node(3));
    let short = p.utility(p.node(7), p.node(5));
    assert!((slow - 1. / 20.).abs() < 1e-6);
    assert!((short - 1. / 10.8).abs() < 1e-6);
    assert!(short > slow);
    let hot = p.hot_node_search(&S::new([0., 0.]), 0.6, 0.).unwrap();
    assert_eq!(hot.node_id, 7);
    assert_eq!(p.nearest_eligible_neighbor(p.node(7), &hot.feasible_neighbors_of_different_tree), Some(5));
}

#[test]
fn two_existing_routes_pick_lower_total_length_not_nearest_first_node() {
    let mut p = two_routes();
    let path = p.build_planned_path().unwrap();
    assert_eq!(path, vec![5, 4, 0]);
    assert!((p.current_state.euclidean_distance(p.node(path[0]).state())
        + p.node(path[0]).cumulative_cost - 10.8).abs() < 1e-5);
}

#[test]
fn utility_uses_nearest_neighbors_membership_and_connection_length() {
    let p = planner(&[
        ([10., 0.], None, 0), ([2., 4.], Some(0), 0), ([2., 0.], Some(1), 0),
        ([1., 0.], None, 1),
    ], [0., 0.]);
    let actual_cost = p.node(2).cumulative_cost;
    assert!((p.utility(p.node(3), p.node(2)) - 1. / (1. + 1. + actual_cost)).abs() < 1e-6);
    // Here n is MAIN but its nearest eligible neighbor n' is detached: Eq.(2)
    // uses the neighbor's Euclidean goal estimate, not n's cumulative cost.
    assert!((p.utility(p.node(2), p.node(3)) - 1. / (2. + 1. + 9.)).abs() < 1e-6);
}

#[test]
fn nearest_eligible_excludes_blocked_and_pruned_neighbors() {
    let mut p = planner(&[
        ([10., 0.], None, 0), ([0., 0.], None, 1),
        ([0.1, 0.], None, -1), ([0.6, 0.], None, 2), ([0., 0.9], None, 3),
    ], [0., -1.]);
    p.set_static_validity_checker(Box::new(AnalyticValidityChecker::new(vec![
        Sphere::new(S::new([0.3, 0.]), 0.1)])));
    let eligible = p.get_feasible_neighbors_of_different_tree(p.node(1));
    assert!(!eligible.contains(&2)); assert!(!eligible.contains(&3));
    assert_eq!(p.nearest_eligible_neighbor(p.node(1), &eligible), Some(4));
}

#[test]
fn informed_repair_connects_one_nearest_then_recomputes_eligibility() {
    let mut p = planner(&[
        ([10., 0.], None, 0), ([0., 0.], None, 1),
        ([0.5, 0.], None, 2), ([1., 0.], None, 3),
    ], [0., -1.]);
    let neighbors = p.get_feasible_neighbors_of_different_tree(p.node(1));
    let hot = HotNode {node_id: 1, feasible_neighbors_of_different_tree: neighbors, utility: 1.};
    p.reconnect_best_hot_node(&hot);
    assert_eq!(p.node(1).tree_id, p.node(2).tree_id);
    assert_ne!(p.node(1).tree_id, p.node(3).tree_id);
    assert!(p.node(3).parent.is_none());
    let new_neighbors = p.get_feasible_neighbors_of_different_tree(p.node(1));
    assert!(!new_neighbors.contains(&2));
    assert_eq!(p.nearest_eligible_neighbor(p.node(1), &new_neighbors), Some(3));
    assert!(p.debug_check_finalized_forest());
}

#[test]
fn standard_sampling_still_connects_all_nearby_reachable_subtrees() {
    let mut p = planner(&[
        ([10., 0.], None, 0), ([-0.5, 0.], None, 1), ([0.5, 0.], None, 2),
    ], [0., -1.]);
    assert!(p.add_new_node_at_state_and_connect(S::new([0., 0.])));
    assert_eq!(p.node(1).tree_id, p.node(2).tree_id);
    assert_eq!(p.node(3).tree_id, p.node(1).tree_id);
    assert!(p.debug_check_finalized_forest());
}

#[test]
fn lsr_shell_uses_geometry_not_cumulative_cost_and_includes_first_center() {
    let mut p = planner(&[
        ([0., 0.], None, 0), ([0.5, 0.], None, 1),
    ], [0., -1.]);
    // The goal node has cumulative cost zero, but remains a hot-node at the
    // center of the first ball. The old cumulative-cost skip omitted it.
    assert_eq!(p.hot_node_search(&S::new([0., 0.]), 0.25, 0.).unwrap().node_id, 0);
    // Both nodes have now been searched spatially; an infinite detached cost
    // must not cause the same node to reappear in a later shell.
    assert!(p.hot_node_search(&S::new([0., 0.]), 1., 0.5).is_none());
}

#[test]
fn expansion_searches_final_partial_shell_but_not_beyond_maximum() {
    let mut p = planner(&[
        ([20., 0.], None, 0), ([9., 0.], None, 1), ([9.5, 0.], None, 2),
    ], [0., 0.]);
    let hot = p.find_hot_node_with_expanding_lsr(&S::new([0., 0.])).unwrap();
    assert!([1, 2].contains(&hot.node_id));
    let mut outside = planner(&[
        ([20., 0.], None, 0), ([10.1, 0.], None, 1), ([10.6, 0.], None, 2),
    ], [0., 0.]);
    assert!(outside.find_hot_node_with_expanding_lsr(&S::new([0., 0.])).is_none());
}
