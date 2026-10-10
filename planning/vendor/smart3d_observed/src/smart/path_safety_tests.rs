//! Deterministic regression tests for the local execution-path predicate.
//! No random RRT initialization, simulator, external map or fake trigger obstacle.
use super::*;
use crate::obstacles::AnalyticValidityChecker;
use crate::rrt::{LinearNearestNeighbors, UniformDistribution};
use crate::rrt::termination::MaxIterationsTermination;

type State = RealVectorState<f32, 2>;
type Planner = SMART<f32, 2, LinearNearestNeighbors<f32, 2>>;

fn chain(points: &[[f32; 2]], lrz: f32) -> Planner {
    let mut nodes: Vec<rrt_star::Node<f32, 2>> = Vec::new();
    let mut neighbors = LinearNearestNeighbors::new();
    let mut cost = 0.0;
    for (id, xy) in points.iter().rev().enumerate() {
        let state = State::new(*xy);
        if id > 0 {
            cost += state.euclidean_distance(nodes[id - 1].state());
        }
        neighbors.add(state, id);
        nodes.push(rrt_star::Node::new(state, if id == 0 { None } else { Some(id - 1) }, cost));
    }
    let checker = AnalyticValidityChecker::<f32, 2, StaticSphericalObstacle<f32, 2>>::new(vec![]);
    let mut planner = SMART::new_from_initial_rrt_star_tree(
        &nodes, neighbors, State::new(points[0]), State::new(*points.last().unwrap()),
        0.01, Box::new(checker), Box::new(UniformDistribution::new([(-20., 30.), (-20., 30.)])),
        lrz, 1.0, 1.5, 10.0, 1.7, 0.01, false, 0.01,
    );
    planner.initial_plan_build_path();
    assert_eq!(planner.get_path().unwrap(), points.iter().map(|p| State::new(*p)).collect::<Vec<_>>());
    planner
}

fn line(last: usize) -> Vec<[f32; 2]> {
    (0..=last).map(|x| [x as f32, 0.]).collect()
}

fn circle(center: [f32; 2], radius: f32) -> Vec<SphericalObstacleWithOhz<f32, 2>> {
    let obstacle = StaticSphericalObstacle::new(State::new(center), radius);
    vec![SphericalObstacleWithOhz::new(obstacle.clone(), obstacle)]
}

fn topology(planner: &Planner) -> Vec<(State, Option<usize>, i32)> {
    planner.get_nodes().iter().map(|n| (n.state, n.parent, n.tree_id)).collect()
}

fn assert_native_repair_triggered(planner: &mut Planner, next: usize, obstacles: &[SphericalObstacleWithOhz<f32, 2>]) {
    // A zero sampling budget allows a bounded failure. The requirement here is
    // that native CPR repair runs, rather than returning NoReplanningNeeded.
    let result = planner.update(planner.current_state, next, &obstacles.to_vec(),
        Some(&mut MaxIterationsTermination::new(0)));
    assert!(result.replanned(), "unexpected native result: {result}");
    assert!(matches!(planner.replanning_triggered_by(), Some(ReplanningTriggerCondition::PathUnsafe)));
    assert!(planner.pruned_nodes.is_empty());
    assert!(planner.disjoint_tree_roots.is_empty());
}

#[test]
fn outgoing_next_edge_interior_triggers_native_repair() {
    let mut planner = chain(&line(3), 9.0);
    let obstacles = circle([1.5, 0.2], 0.3);
    planner.build_cpr(&obstacles);
    assert!(!planner.is_path_safe_within_lrz(1).0);
    assert_native_repair_triggered(&mut planner, 1, &obstacles);
}

#[test]
fn lrz_boundary_crossing_edge_triggers_native_repair() {
    let mut planner = chain(&line(10), 9.0);
    let obstacles = circle([8.8, 0.4], 0.5);
    planner.build_cpr(&obstacles);
    assert!(!planner.is_path_safe_within_lrz(0).0);
    assert_native_repair_triggered(&mut planner, 0, &obstacles);
}

#[test]
fn measured_position_to_next_node_is_checked() {
    let mut planner = chain(&line(3), 9.0);
    let obstacles = circle([0.5, 0.2], 0.3);
    planner.build_cpr(&obstacles);
    assert!(!planner.is_path_safe_within_lrz(1).0);
}

#[test]
fn segment_with_both_endpoints_outside_lrz_is_checked_inside() {
    let mut planner = chain(&[[0., 0.], [0., 12.], [12., 12.], [12., -12.], [-12., 12.]], 9.0);
    planner.build_cpr(&circle([3., -3.], 0.5));
    assert!(!planner.is_path_safe_within_lrz(1).0);
}

#[test]
fn far_obstacle_does_not_change_tree_or_path() {
    let mut planner = chain(&line(20), 9.0);
    let before = topology(&planner);
    let path = planner.get_path();
    let result = planner.update(State::new([0., 0.]), 1, &circle([15., 0.], 0.5),
        Some(&mut MaxIterationsTermination::new(0)));
    assert!(matches!(result, SmartUpdateResult::NoReplanningNeeded));
    assert_eq!(topology(&planner), before);
    assert_eq!(planner.get_path(), path);
}

#[test]
fn cpr_obstacle_intersection_only_outside_lrz_does_not_trigger() {
    let mut planner = chain(&line(11), 9.0);
    // Its OHZ overlaps the LRZ, but the straight path meets it only beyond x=9.
    let obstacles = circle([9.6, 1.0], 1.1);
    planner.build_cpr(&obstacles);
    assert_eq!(planner.cpr_obstacles.len(), 1);
    assert!(planner.is_path_safe_within_lrz(0).0);
    let before = topology(&planner);
    let result = planner.update(State::new([0., 0.]), 0, &obstacles,
        Some(&mut MaxIterationsTermination::new(0)));
    assert!(matches!(result, SmartUpdateResult::NoReplanningNeeded));
    assert_eq!(topology(&planner), before);
}

#[test]
fn already_traversed_edges_are_not_checked_again() {
    let mut planner = chain(&line(12), 9.0);
    let before = topology(&planner);
    let result = planner.update(State::new([5.5, 0.]), 6, &circle([4.5, 0.2], 0.3),
        Some(&mut MaxIterationsTermination::new(0)));
    assert!(matches!(result, SmartUpdateResult::NoReplanningNeeded));
    assert_eq!(topology(&planner), before);
}

#[test]
fn absent_empty_or_out_of_range_cursor_is_unsafe_without_panicking() {
    let mut planner = chain(&line(3), 9.0);
    assert_eq!(planner.is_path_safe_within_lrz(4), (false, None));
    assert_eq!(planner.is_path_safe_within_lrz(usize::MAX), (false, None));
    planner.current_path = Some(vec![]);
    assert_eq!(planner.is_path_safe_within_lrz(0), (false, None));
    planner.current_path = None;
    assert_eq!(planner.is_path_safe_within_lrz(0), (false, None));
}

#[test]
fn valid_final_cursor_and_zero_length_segments_are_supported() {
    let mut planner = chain(&line(3), 9.0);
    planner.current_state = State::new([3., 0.]);
    planner.build_cpr(&circle([1., 0.], 0.3));
    assert!(planner.is_path_safe_within_lrz(3).0);
    // Repeated path entries do not cause division by zero or skip later edges.
    let ids = planner.current_path.as_ref().unwrap().clone();
    planner.current_state = State::new([0., 0.]);
    planner.current_path = Some(vec![ids[0], ids[0], ids[1], ids[2], ids[3]]);
    planner.build_cpr(&circle([1.5, 0.2], 0.3));
    assert!(!planner.is_path_safe_within_lrz(0).0);
}

#[test]
fn lrz_tangent_contact_with_an_occupied_point_is_not_discarded() {
    let mut planner = chain(&[[0., 0.], [-2., 12.], [-2., 9.], [2., 9.]], 9.0);
    planner.build_cpr(&circle([0., 9.], 0.1));
    assert!(!planner.is_path_safe_within_lrz(1).0);
}

fn sweep(center: [f32; 2], end: [f32; 2], radius: f32) -> Vec<SphericalObstacleWithOhz<f32, 2>> {
    let center=State::new(center);let end=State::new(end);
    vec![SphericalObstacleWithOhz::new(StaticSphericalObstacle::new(center,radius),
        StaticSphericalObstacle::new(center,radius+center.euclidean_distance(&end))).with_sweep(end)]
}

#[test]
fn moving_away_capsule_avoids_spherical_false_path_cut() {
    let points=line(8);let mut directed=chain(&points,9.);
    let before=topology(&directed);let path=directed.get_path();
    let result=directed.update(State::new([0.,0.]),1,&sweep([4.,1.],[4.,3.],0.5),
        Some(&mut MaxIterationsTermination::new(0)));
    assert!(matches!(result,SmartUpdateResult::NoReplanningNeeded));
    assert_eq!(topology(&directed),before);assert_eq!(directed.get_path(),path);
    let mut baseline=chain(&points,9.);
    baseline.build_cpr(&vec![SphericalObstacleWithOhz::new(
        StaticSphericalObstacle::new(State::new([4.,1.]),0.5),
        StaticSphericalObstacle::new(State::new([4.,1.]),2.5))]);
    assert!(!baseline.is_path_safe_within_lrz(1).0);
}

#[test]
fn capsule_crossing_triggers_native_prune_and_repair() {
    let mut planner=chain(&line(8),9.);
    assert_native_repair_triggered(&mut planner,1,&sweep([4.,2.],[4.,-2.],0.5));
}

#[test]
fn capsule_broadphase_never_prunes_nodes_outside_exact_sweep() {
    let mut planner=chain(&[[0.,0.],[1.,0.],[2.,0.],[3.,0.],[4.,0.]],9.);
    // The enclosing sphere includes x=1 and x=3, but the capsule only covers x=2.
    planner.build_cpr(&sweep([2.,1.],[2.,-1.],0.25));
    planner.prune_tree();
    let cut:Vec<_>=planner.pruned_nodes.iter().map(|(id,_)|*planner.node(*id).state()).collect();
    assert_eq!(cut,vec![State::new([2.,0.])]);
}

#[test]
fn capsule_selection_includes_lrz_overlap_and_checks_long_crossing_edges() {
    let mut planner=chain(&line(8),9.);
    // Present disk is outside LRZ, but the directed prediction enters it.
    planner.build_cpr(&sweep([12.,0.],[8.,0.],0.5));
    assert_eq!(planner.cpr_obstacles.len(),1);
    assert!(!planner.is_path_safe_within_lrz(0).0);
    assert!(!planner.cpr_validity_check_edge(&State::new([-50.,0.]),&State::new([50.,0.])));
}

#[test]
fn robot_in_directional_prediction_stays_blocked_without_shrinking() {
    let mut planner=chain(&line(8),9.);
    let obstacles=sweep([-2.,0.],[2.,0.],0.5);
    planner.build_cpr(&obstacles);
    assert_eq!(planner.robot_inside_ohzs.len(),1);
    assert!(!planner.cpr_validity_check_state(&State::new([0.,0.])));
    assert_native_repair_triggered(&mut planner,1,&obstacles);
}

fn ellipse(center: [f32; 2], end: [f32; 2], radius: f32) -> Vec<SphericalObstacleWithOhz<f32, 2>> {
    let center=State::new(center);let end=State::new(end);
    vec![SphericalObstacleWithOhz::new(StaticSphericalObstacle::new(center,radius),
        StaticSphericalObstacle::new(center,radius+center.euclidean_distance(&end))).with_ellipse(end)]
}

#[test]
fn ellipse_moving_away_preserves_path_but_crossing_triggers_native_repair() {
    let points=line(8);let mut planner=chain(&points,9.);
    let before=topology(&planner);let path=planner.get_path();
    let result=planner.update(State::new([0.,0.]),1,&ellipse([4.,1.],[4.,3.],0.5),
        Some(&mut MaxIterationsTermination::new(0)));
    assert!(matches!(result,SmartUpdateResult::NoReplanningNeeded));
    assert_eq!(topology(&planner),before);assert_eq!(planner.get_path(),path);
    assert_native_repair_triggered(&mut planner,1,&ellipse([4.,2.],[4.,-2.],0.5));
}

#[test]
fn ellipse_broadphase_does_not_replace_narrowphase_in_native_pruning() {
    let mut planner=chain(&[[0.,0.],[1.,0.],[2.,0.],[3.,0.],[4.,0.]],9.);
    // a=1.25 includes x=1,3 in the bound; b=sqrt(.3125) excludes them.
    planner.build_cpr(&ellipse([2.,1.],[2.,-1.],0.25));
    planner.prune_tree();
    let cut:Vec<_>=planner.pruned_nodes.iter().map(|(id,_)|*planner.node(*id).state()).collect();
    assert_eq!(cut,vec![State::new([2.,0.])]);
}

#[test]
fn ellipse_lrz_overlap_long_crossing_and_robot_inside_use_same_hazard() {
    let mut planner=chain(&line(8),9.);
    planner.build_cpr(&ellipse([12.,0.],[8.,0.],0.5));
    assert_eq!(planner.cpr_obstacles.len(),1);
    assert!(!planner.is_path_safe_within_lrz(0).0);
    assert!(!planner.cpr_validity_check_edge(&State::new([-50.,0.]),&State::new([50.,0.])));
    let occupied=ellipse([-2.,0.],[2.,0.],0.5);
    planner.build_cpr(&occupied);
    assert_eq!(planner.robot_inside_ohzs.len(),1);
    assert!(!planner.cpr_validity_check_state(&State::new([0.,0.])));
    assert!(!planner.cpr_validity_check_state(&State::new([2.5,0.])));
    assert_native_repair_triggered(&mut planner,1,&occupied);
}
