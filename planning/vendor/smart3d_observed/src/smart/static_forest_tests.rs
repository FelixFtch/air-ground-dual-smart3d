//! Permanent static increments interleaved with native dynamic CPR updates.
use super::*;
use crate::obstacles::AnalyticValidityChecker;
use crate::rrt::{LinearNearestNeighbors, UniformDistribution};
use crate::rrt::termination::MaxIterationsTermination;
use std::sync::{Arc, atomic::{AtomicUsize, Ordering}};

type S = RealVectorState<f32, 2>;
type P = SMART<f32, 2, LinearNearestNeighbors<f32, 2>>;
type Circle = StaticSphericalObstacle<f32, 2>;
type Check = AnalyticValidityChecker<f32, 2, Circle>;

fn sphere(x: f32, y: f32, radius: f32) -> Circle { Circle::new(S::new([x, y]), radius) }
fn checker(walls: &[Circle]) -> Check { Check::new(walls.to_vec()) }
fn line(with_detour: bool) -> P {
    let mut nodes = Vec::<rrt_star::Node<f32, 2>>::new();
    let mut nn = LinearNearestNeighbors::new();
    for id in 0..=6 {
        let s = S::new([(6 - id) as f32, 0.]);
        nn.add(s, id);
        nodes.push(rrt_star::Node::new(s, if id == 0 { None } else { Some(id - 1) }, id as f32));
    }
    if with_detour {
        for (j, x) in [6., 4., 2., 0.].into_iter().enumerate() {
            let id = nodes.len(); let s = S::new([x, 2.]); nn.add(s, id);
            nodes.push(rrt_star::Node::new(s, Some(if j == 0 { 0 } else { id - 1 }), (j + 1) as f32 * 2.));
        }
    }
    let mut p = SMART::new_from_initial_rrt_star_tree(&nodes, nn, S::new([0., 0.]), S::new([6., 0.]),
        0.01, Box::new(checker(&[])), Box::new(UniformDistribution::new([(-5., 15.), (-5., 5.)])),
        9., 1., 1.5, 10., 1.7, 0.01, false, 0.01);
    p.initial_plan_build_path(); p
}
fn apply(p: &mut P, all: &[Circle], new: &[Circle], actual: [f32; 2]) -> SmartUpdateResult {
    p.update_new_static_obstacles(Box::new(checker(all)), &checker(new),
        Some(&mut MaxIterationsTermination::new(0)), S::new(actual))
}

fn drain_clear(p: &mut P) -> (usize,usize) {
    let mut restored=0;let mut connected=0;
    for _ in 0..100 {
        let r=p.maintain_static_clear_regions(1,1,1,Duration::from_secs(1));
        assert!(r.nodes_examined<=1 && r.regions_examined<=1 && r.connections<=1);
        restored+=r.restored_nodes;connected+=r.connections;
        if r.pending_nodes+r.pending_regions==0 {return (restored,connected);}
    }
    panic!("bounded clear queue failed to drain");
}

#[test]
fn map_clear_restores_same_samples_outside_lrz_while_actual_still_in_collision() {
    let mut p=line(false);p.lrz_radius=0.5;
    let foot=sphere(0.,0.,0.2);let wall=sphere(3.,0.,0.2);
    let count=p.nodes.len();
    assert!(matches!(apply(&mut p,&[foot.clone(),wall.clone()],&[foot.clone(),wall],[0.,0.]),SmartUpdateResult::InCollision));
    assert_eq!(p.node(3).tree_id,P::PRUNED_TREE_ID);
    p.set_static_validity_checker(Box::new(checker(&[foot.clone()])));
    p.queue_static_clear_regions(&[(S::new([3.,0.]),0.5)]);
    let (restored,connections)=drain_clear(&mut p);
    assert_eq!(restored,1);assert!(connections>=2);
    assert_eq!(p.nodes.len(),count);assert_eq!(p.node(3).tree_id,P::MAIN_TREE_ID);
    assert_eq!(p.node(5).tree_id,P::MAIN_TREE_ID);
    assert_eq!(p.node(6).tree_id,P::PRUNED_TREE_ID);
    assert!(p.get_path().is_none(),"maintenance must not approve a motion path");
    assert_static_forest(&p,&[foot]);
    let r=p.maintain_static_clear_regions(32,128,16,Duration::from_millis(8));
    assert_eq!(r.nodes_examined+r.regions_examined+r.connections+r.restored_nodes,0);
}

#[test]
fn clear_maintenance_keeps_remaining_walls_and_existing_main_path() {
    let mut p=line(true);let wall=sphere(3.,0.,0.2);
    assert!(apply(&mut p,&[wall.clone()],&[wall.clone()],[0.,0.]).is_success());
    let path=p.current_path.clone();let main_parents:Vec<_>=p.nodes.iter().enumerate()
        .filter(|(_,n)|n.tree_id==0).map(|(id,n)|(id,n.parent)).collect();
    // An unrelated raw clear cannot remove a remaining semantic/manual wall.
    p.queue_static_clear_regions(&[(S::new([3.,0.]),1.)]);
    assert_eq!(drain_clear(&mut p).0,0);assert_eq!(p.node(3).tree_id,-1);
    assert_static_forest(&p,&[wall]);
    p.set_static_validity_checker(Box::new(checker(&[])));
    p.queue_static_clear_regions(&[(S::new([3.,0.]),1.)]);
    assert_eq!(drain_clear(&mut p).0,1);
    assert_eq!(p.current_path,path);
    for (id,parent) in main_parents {assert_eq!(p.node(id).parent,parent);}
    assert_static_forest(&p,&[]);
}

#[test]
fn queued_clear_checks_latest_obstacles_before_restoring() {
    let mut p=line(false);let wall=sphere(3.,0.,0.2);
    apply(&mut p,&[wall.clone()],&[wall.clone()],[0.,0.]);
    p.set_static_validity_checker(Box::new(checker(&[])));
    for _ in 0..100 {p.queue_static_clear_regions(&[(S::new([3.,0.]),1.)]);}
    let queued=p.maintain_static_clear_regions(0,0,0,Duration::from_millis(8));
    assert_eq!(queued.pending_regions,1,"repeated events deduplicate");
    p.set_static_validity_checker(Box::new(checker(&[wall.clone()])));
    assert_eq!(drain_clear(&mut p).0,0);
    assert_static_forest(&p,&[wall]);
}
fn dynamic(p: &mut P, obstacles: &[Circle]) -> SmartUpdateResult {
    let observations = obstacles.iter().map(|s| SphericalObstacleWithOhz::new(s.clone(), s.clone())).collect();
    p.update(S::new([0., 0.]), 0, &observations, Some(&mut MaxIterationsTermination::new(0)))
}
fn assert_static_forest(p: &P, walls: &[Circle]) {
    assert!(p.pruned_nodes.is_empty()); assert!(p.disjoint_tree_roots.is_empty());
    assert!(p.optimization_start_nodes.is_empty()); assert!(p.debug_check_finalized_forest());
    let check = checker(walls);
    for n in p.get_nodes() {
        if n.tree_id == P::PRUNED_TREE_ID {
            assert!(!check.is_state_valid(&n.state));
            assert!(n.parent.is_none()); assert!(n.children.is_empty());
        } else {
            assert!(check.is_state_valid(&n.state));
            if let Some(parent) = n.parent {
                assert!(check.is_edge_valid(&p.node(parent).state, &n.state));
            }
        }
    }
    if let Some(path) = p.get_path() {
        assert_eq!(path.last(), Some(&p.goal));
        assert!(check.is_edge_valid(&p.current_state, &path[0]));
        assert!(path.windows(2).all(|e| check.is_edge_valid(&e[0], &e[1])));
    }
}

#[test]
fn dynamic_pruning_of_permanent_forest_root_does_not_panic_or_resurrect_wall() {
    let mut p = line(false); let wall = sphere(3., 0., 0.2);
    assert!(matches!(apply(&mut p, &[wall.clone()], &[wall.clone()], [0., 0.]), SmartUpdateResult::ReplanningFailed));
    assert_eq!(p.node(3).tree_id, P::PRUNED_TREE_ID);
    assert_eq!(p.node(4).parent, None);
    for _ in 0..20 {
        assert!(matches!(dynamic(&mut p, &[sphere(2., 0., 0.3)]), SmartUpdateResult::ReplanningFailed));
        assert_static_forest(&p, &[wall.clone()]);
        assert_eq!(p.node(3).tree_id, P::PRUNED_TREE_ID);
        assert_eq!(p.get_nodes().len(), 7);
    }
}

#[test]
fn empty_delta_failed_path_retries_instead_of_false_no_replan() {
    let mut p = line(false); let wall = sphere(3., 0., 0.2);
    apply(&mut p, &[wall.clone()], &[wall.clone()], [0., 0.]);
    assert!(p.get_path().is_none());
    assert!(matches!(apply(&mut p, &[wall.clone()], &[], [0., 0.]), SmartUpdateResult::ReplanningFailed));
    assert!(matches!(dynamic(&mut p, &[]), SmartUpdateResult::ReplanningFailed));
    assert_static_forest(&p, &[wall]);
}

#[test]
fn static_edge_interior_stays_cut_after_dynamic_restore() {
    let mut p = line(false); let wall = sphere(2.5, 0., 0.2);
    apply(&mut p, &[wall.clone()], &[wall.clone()], [0., 0.]);
    assert!(p.nodes.iter().all(|n| n.tree_id != P::PRUNED_TREE_ID));
    assert_eq!(p.node(4).parent, None);
    for _ in 0..5 {
        dynamic(&mut p, &[sphere(2., 0., 0.3)]);
        assert_static_forest(&p, &[wall.clone()]);
        assert_eq!(p.node(4).parent, None);
    }
}

#[test]
fn successive_static_components_keep_unique_ids_then_survive_dynamic_cycles() {
    let mut p = line(false); let a = sphere(3., 0., 0.2); let b = sphere(5., 0., 0.2);
    apply(&mut p, &[a.clone()], &[a.clone()], [0., 0.]);
    let prior = p.tree_id_counter;
    apply(&mut p, &[a.clone(), b.clone()], &[b.clone()], [0., 0.]);
    assert!(p.tree_id_counter > prior);
    for obs in [vec![], vec![sphere(2., 0., 0.3)], vec![sphere(4., 0., 0.3)]] {
        dynamic(&mut p, &obs);
        assert_static_forest(&p, &[a.clone(), b.clone()]);
    }
}

#[test]
fn finalizer_reconnection_updates_path_and_result_on_same_tree() {
    let mut p = line(true); let wall = sphere(3., 0., 0.3);
    let count = p.nodes.len();
    let result = apply(&mut p, &[wall.clone()], &[wall.clone()], [0., 0.]);
    assert!(matches!(result, SmartUpdateResult::ReplanningSuccessful));
    assert!(p.get_path().is_some()); assert_eq!(p.nodes.len(), count);
    assert_static_forest(&p, &[wall.clone()]);
    let result = dynamic(&mut p, &[]);
    assert!(matches!(result, SmartUpdateResult::NoReplanningNeeded));
    assert_static_forest(&p, &[wall]);
}

#[test]
fn successful_static_forest_reuses_tree_after_dynamic_block_and_clear() {
    let mut p = line(true); let wall = sphere(3., 0., 0.3);
    assert!(matches!(apply(&mut p, &[wall.clone()], &[wall.clone()], [0., 0.]), SmartUpdateResult::ReplanningSuccessful));
    let count = p.nodes.len();
    for _ in 0..10 {
        assert!(dynamic(&mut p, &[sphere(1., 0., 0.2)]).replanned());
        assert_static_forest(&p, &[wall.clone()]);
        assert!(dynamic(&mut p, &[]).is_success());
        assert!(p.get_path().is_some());
        assert_static_forest(&p, &[wall.clone()]);
        assert_eq!(p.nodes.len(), count);
    }
}

#[test]
fn newly_occupied_current_or_goal_commits_all_static_cuts_before_failure() {
    for x in [0., 6.] {
        let mut p = line(false); let walls = [sphere(x, 0., 0.2), sphere(3., 0., 0.2)];
        assert!(matches!(apply(&mut p, &walls, &walls, [0., 0.]), SmartUpdateResult::InCollision));
        assert!(p.get_path().is_none());
        assert_eq!(p.node(3).tree_id, P::PRUNED_TREE_ID);
        assert_eq!(p.node((6. - x) as usize).tree_id, P::PRUNED_TREE_ID);
        assert_static_forest(&p, &walls);
        assert!(matches!(dynamic(&mut p, &[]), SmartUpdateResult::InCollision));
        assert!(p.get_path().is_none()); assert_static_forest(&p, &walls);
    }
}

#[test]
fn full_static_entrypoint_has_same_failure_and_forest_contract() {
    let mut p = line(false); let wall = sphere(3., 0., 0.2);
    assert!(matches!(p.update_static_obstacles(Box::new(checker(&[wall.clone()])),
        Some(&mut MaxIterationsTermination::new(0)), S::new([0., 0.])), SmartUpdateResult::ReplanningFailed));
    dynamic(&mut p, &[sphere(2., 0., 0.3)]);
    assert_static_forest(&p, &[wall]);
}

struct CountChecker { inner: Check, state_calls: Arc<AtomicUsize>, edge_calls: Arc<AtomicUsize> }
impl ValidityChecker<f32, 2> for CountChecker {
    fn is_state_valid(&self, s: &S) -> bool { self.state_calls.fetch_add(1, Ordering::Relaxed); self.inner.is_state_valid(s) }
    fn is_edge_valid(&self, a: &S, b: &S) -> bool { self.edge_calls.fetch_add(1, Ordering::Relaxed); self.inner.is_edge_valid(a, b) }
}

#[test]
fn repeated_safe_dynamic_updates_check_only_remaining_path_not_static_tree() {
    let mut p = line(true); let wall = sphere(3., 0., 0.3);
    apply(&mut p, &[wall.clone()], &[wall.clone()], [0., 0.]);
    let states = Arc::new(AtomicUsize::new(0)); let edges = Arc::new(AtomicUsize::new(0));
    p.set_static_validity_checker(Box::new(CountChecker { inner: checker(&[wall]), state_calls: states.clone(), edge_calls: edges.clone() }));
    let topology: Vec<_> = p.nodes.iter().map(|n| (n.parent, n.tree_id)).collect();
    let remaining = p.current_path.as_ref().unwrap().len();
    for _ in 0..50 { assert!(matches!(dynamic(&mut p, &[]), SmartUpdateResult::NoReplanningNeeded)); }
    assert_eq!(states.load(Ordering::Relaxed), 50 * (2 + remaining));
    assert_eq!(edges.load(Ordering::Relaxed), 50 * remaining);
    assert_eq!(p.nodes.iter().map(|n| (n.parent, n.tree_id)).collect::<Vec<_>>(), topology);
}

#[test]
fn new_static_connector_collision_repairs_before_any_raw_grid_or_dynamic_update() {
    let mut p = line(false); p.path_node_radius = 1.7;
    let original_path = p.get_path();
    let wall = sphere(0.7, 0.74, 0.727);
    let check = checker(&[wall.clone()]);
    // A 4 cm measured lateral offset is enough for the new connector to be
    // blocked while every retained arena edge on y=0 remains static-valid.
    for n in &p.nodes {
        assert!(check.is_state_valid(&n.state));
        if let Some(parent) = n.parent { assert!(check.is_edge_valid(&p.node(parent).state, &n.state)); }
    }
    let actual = S::new([0.5, 0.04]);
    assert!(check.is_state_valid(&actual));
    assert!(!check.is_edge_valid(&actual, &S::new([1., 0.])));
    let result = p.update_new_static_obstacles_with_cursor(Box::new(checker(&[wall.clone()])),
        &checker(&[wall.clone()]), Some(&mut MaxIterationsTermination::new(0)), actual, 1);
    assert!(matches!(result, SmartUpdateResult::ReplanningSuccessful));
    assert_eq!(p.get_path(), original_path); // coordinates alone cannot identify a new execution
    assert!(p.is_remaining_path_static_valid(0));
    assert!(!p.is_remaining_path_static_valid(1)); // returned replanned path must start at zero
    assert_static_forest(&p, &[wall]);
}

#[test]
fn with_cursor_does_not_recheck_a_blocked_already_traversed_connector() {
    let mut p = line(false); p.path_node_radius = 1.7;
    let original_path = p.get_path();
    let wall = sphere(0.3, 0.74, 0.727);
    let actual = S::new([0.5, 0.04]); let check = checker(&[wall.clone()]);
    assert!(!check.is_edge_valid(&actual, &S::new([0., 0.])));
    assert!(check.is_edge_valid(&actual, &S::new([1., 0.])));
    let result = p.update_new_static_obstacles_with_cursor(Box::new(check), &checker(&[wall]),
        Some(&mut MaxIterationsTermination::new(0)), actual, 1);
    assert!(matches!(result, SmartUpdateResult::NoReplanningNeeded));
    assert_eq!(p.get_path(), original_path);
    assert!(p.is_remaining_path_static_valid(1));
}

#[test]
fn far_static_path_edge_blocks_no_replan_even_outside_dynamic_lrz() {
    let mut p = line(true); p.lrz_radius = 1.;
    // A retained shortcut may cross a newly known cell without any arena
    // parent edge crossing it. No delta arena cuts are needed in this case.
    p.current_path = Some(vec![6, 7, 0]);
    let previous = p.current_path.clone(); let wall = sphere(4.5, 1.5, 0.2);
    let check = checker(&[wall.clone()]);
    for n in &p.nodes {
        assert!(check.is_state_valid(&n.state));
        if let Some(parent) = n.parent { assert!(check.is_edge_valid(&n.state, p.node(parent).state())); }
    }
    p.set_static_validity_checker(Box::new(check));
    assert!(p.is_path_safe_within_lrz(0).0);
    assert!(matches!(dynamic(&mut p, &[]), SmartUpdateResult::ReplanningSuccessful));
    assert_ne!(p.current_path, previous);
    assert_static_forest(&p, &[wall]);
}

#[test]
fn builder_tries_next_existing_candidate_when_cheapest_parent_chain_is_dirty() {
    let mut p = line(true); p.path_node_radius = 2.1;
    let wall = sphere(3., 0., 0.3); let check = checker(&[wall.clone()]);
    p.set_static_validity_checker(Box::new(check));
    let path = p.build_planned_path().expect("existing upper branch remains usable");
    assert_eq!(path, vec![10, 9, 8, 7, 0]);
    let check = checker(&[wall]);
    assert!(path.windows(2).all(|e| check.is_edge_valid(p.node(e[0]).state(), p.node(e[1]).state())));
}

#[test]
fn dynamic_static_check_respects_actual_connector_and_traversed_cursor() {
    for (x, expected_no_replan) in [(0.7, false), (0.3, true)] {
        let mut p = line(false); p.path_node_radius = 1.7;
        let wall = sphere(x, 0.74, 0.727);
        p.set_static_validity_checker(Box::new(checker(&[wall])));
        let result = p.update(S::new([0.5, 0.04]), 1, &vec![],
            Some(&mut MaxIterationsTermination::new(0)));
        assert_eq!(matches!(result, SmartUpdateResult::NoReplanningNeeded), expected_no_replan);
        assert!(p.is_remaining_path_static_valid(if expected_no_replan { 1 } else { 0 }));
    }
}

#[test]
fn offpath_known_delta_cuts_tree_without_rebuilding_path_or_cursor() {
    let mut p = line(true);
    let original = p.current_path.clone(); let original_edges: Vec<_> =
        original.as_ref().unwrap().iter().map(|&id| p.node(id).parent).collect();
    let wall = sphere(4., 2., 0.3);
    let result = p.update_new_static_obstacles_with_cursor(Box::new(checker(&[wall.clone()])),
        &checker(&[wall.clone()]), Some(&mut MaxIterationsTermination::new(0)), S::new([0.5, 0.]), 1);
    assert!(matches!(result, SmartUpdateResult::NoReplanningNeeded));
    assert_eq!(p.current_path, original);
    assert_eq!(p.current_path.as_ref().unwrap().iter().map(|&id| p.node(id).parent).collect::<Vec<_>>(), original_edges);
    assert_eq!(p.node(8).tree_id, P::PRUNED_TREE_ID);
    assert!(p.is_remaining_path_static_valid(1));
    assert_static_forest(&p, &[wall]);
}

#[test]
fn clearing_temporary_goal_block_restores_only_goal_then_repairs_same_tree() {
    let mut p = line(false); let wall = sphere(6., 0., 0.2); let count = p.nodes.len();
    for _ in 0..10 {
        assert!(matches!(apply(&mut p, &[wall.clone()], &[wall.clone()], [0., 0.]), SmartUpdateResult::InCollision));
        assert_eq!(p.node(0).tree_id, P::PRUNED_TREE_ID);
        assert!(p.node(0).children.is_empty());
        assert_static_forest(&p, &[wall.clone()]);
        p.set_static_validity_checker(Box::new(checker(&[])));
        assert_eq!(p.node(0).tree_id, P::MAIN_TREE_ID);
        assert_eq!(p.node(0).cumulative_cost, 0.);
        assert_eq!(p.node(0).edge_cost, 0.);
        assert!(p.node(0).parent.is_none()); assert!(p.node(0).children.is_empty());
        assert!(p.get_path().is_none()); // clearing alone does not certify a route
        assert!(matches!(p.update(S::new([0., 0.]), 0, &vec![],
            Some(&mut MaxIterationsTermination::new(10))), SmartUpdateResult::ReplanningSuccessful));
        assert_static_forest(&p, &[]);
        assert_eq!(p.nodes.len(), count);
    }
}

#[test]
fn clearing_other_known_cells_cannot_restore_a_permanently_forbidden_goal() {
    let mut p = line(false); let goal_wall = sphere(6., 0., 0.2);
    let temporary_wall = sphere(3., 0., 0.2);
    let all = [goal_wall.clone(), temporary_wall.clone()];
    apply(&mut p, &all, &all, [0., 0.]);
    p.set_static_validity_checker(Box::new(checker(&[goal_wall.clone()])));
    assert_eq!(p.node(0).tree_id, P::PRUNED_TREE_ID);
    assert_eq!(p.node(3).tree_id, P::PRUNED_TREE_ID); // no bulk node resurrection
    assert!(matches!(dynamic(&mut p, &[]), SmartUpdateResult::InCollision));
    assert!(p.get_path().is_none());
    for n in &p.nodes {
        if n.tree_id == P::PRUNED_TREE_ID {
            assert!(n.parent.is_none()); assert!(n.children.is_empty());
        } else if let Some(parent) = n.parent {
            assert!(checker(&[goal_wall.clone()]).is_edge_valid(&n.state, p.node(parent).state()));
        }
    }
    assert!(p.debug_check_finalized_forest());
}

#[test]
fn mixed_known_removal_and_new_delta_restores_goal_without_old_edges() {
    let mut p = line(true); p.hot_node_neighborhood_radius = 2.1;
    let goal_wall = sphere(6., 0., 0.2);
    apply(&mut p, &[goal_wall.clone()], &[goal_wall], [0., 0.]);
    let new_wall = sphere(3., 0., 0.3);
    let result = apply(&mut p, &[new_wall.clone()], &[new_wall.clone()], [0., 0.]);
    assert!(matches!(result, SmartUpdateResult::ReplanningFailed));
    assert_eq!(p.node(0).tree_id, P::MAIN_TREE_ID);
    assert_static_forest(&p, &[new_wall.clone()]);
    assert!(matches!(p.update(S::new([0., 0.]), 0, &vec![],
        Some(&mut MaxIterationsTermination::new(10))), SmartUpdateResult::ReplanningSuccessful));
    assert_static_forest(&p, &[new_wall]);
    assert_eq!(p.nodes.len(), 11);
}

#[test]
fn rrt_conversion_reconciles_stale_descendant_costs_without_rewiring() {
    // RRT* may have shortened the root->parent connection while the child's
    // old cumulative cost was not propagated. Geometry and parent IDs are valid.
    let nodes = vec![rrt_star::Node::new(S::new([2., 0.]), None, 0.),
        rrt_star::Node::new(S::new([1., 0.]), Some(0), 1.),
        rrt_star::Node::new(S::new([0., 0.]), Some(1), 3.)];
    let mut nn = LinearNearestNeighbors::new();
    for (id, n) in nodes.iter().enumerate() { nn.add(n.state().clone(), id); }
    let mut p = SMART::new_from_initial_rrt_star_tree(&nodes, nn, S::new([0., 0.]), S::new([2., 0.]),
        0.01, Box::new(checker(&[])), Box::new(UniformDistribution::new([(-5., 15.), (-5., 5.)])),
        9., 1., 1.5, 10., 1.7, 0.01, false, 0.01);
    assert_eq!(p.node(2).cumulative_cost, 2.);
    assert!(p.debug_check_costs());
    for (id, old) in nodes.iter().enumerate() {
        assert_eq!(p.node(id).parent, old.parent());
        assert_eq!(p.node(id).state(), old.state());
    }
    p.initial_plan_build_path();
    assert_eq!(p.current_path, Some(vec![2, 1, 0]));
    assert_static_forest(&p, &[]);
}

// Models a budget already consumed by the mandatory obstacle scan. Reset must
// occur after that scan, and only when bounded native repair is actually needed.
struct ScanExpiredBudget {
    states: Arc<AtomicUsize>, edges: Arc<AtomicUsize>,
    reset_counts: Vec<(usize, usize)>, evaluates: usize, allowed_iterations: usize,
}
impl ScanExpiredBudget {
    fn new(allowed_iterations: usize) -> Self {
        Self {states: Arc::new(AtomicUsize::new(0)), edges: Arc::new(AtomicUsize::new(0)),
            reset_counts: Vec::new(), evaluates: 0, allowed_iterations}
    }
    fn counted_checker(&self, walls: &[Circle]) -> CountChecker {
        CountChecker {inner: checker(walls), state_calls: self.states.clone(), edge_calls: self.edges.clone()}
    }
}
impl TerminationCondition for ScanExpiredBudget {
    fn evaluate(&mut self) -> bool {
        self.evaluates += 1;
        self.reset_counts.is_empty() || self.evaluates > self.allowed_iterations
    }
    fn reset(&mut self) {
        self.reset_counts.push((self.states.load(Ordering::Relaxed), self.edges.load(Ordering::Relaxed)));
        self.evaluates = 0;
    }
}

#[test]
fn static_scan_expiry_does_not_starve_same_forest_hot_node_repair() {
    for full_scan in [false, true] {
        let mut p = line(true); p.hot_node_neighborhood_radius = 2.1;
        let goal_wall = sphere(6., 0., 0.2);
        apply(&mut p, &[goal_wall.clone()], &[goal_wall], [0., 0.]);
        let wall = sphere(3., 0., 0.3); let mut term = ScanExpiredBudget::new(10);
        let count = p.nodes.len();
        let result = if full_scan {
            p.update_static_obstacles(Box::new(term.counted_checker(&[wall.clone()])),
                Some(&mut term), S::new([0., 0.]))
        } else {
            p.update_new_static_obstacles_with_cursor(Box::new(checker(&[wall.clone()])),
                &term.counted_checker(&[wall.clone()]), Some(&mut term), S::new([0., 0.]), 0)
        };
        assert!(matches!(result, SmartUpdateResult::ReplanningSuccessful));
        assert_eq!(term.reset_counts.len(), 1);
        // Each original arena node was visited before the clock was reset;
        // full-checker point/edge calls can add further validation reads.
        assert!(term.reset_counts[0].0 >= count);
        assert!(term.reset_counts[0].1 > 0);
        assert!(term.evaluates > 0 && term.evaluates <= 10);
        assert_eq!(p.nodes.len(), count); // existing native hot nodes reconnected
        assert_static_forest(&p, &[wall]);
    }
}

#[test]
fn offpath_no_replan_does_not_reset_or_spend_repair_budget() {
    let mut p = line(true); let before = p.current_path.clone();
    let wall = sphere(4., 2., 0.3); let mut term = ScanExpiredBudget::new(10);
    let result = p.update_new_static_obstacles_with_cursor(Box::new(checker(&[wall.clone()])),
        &term.counted_checker(&[wall.clone()]), Some(&mut term), S::new([0.5, 0.]), 1);
    assert!(matches!(result, SmartUpdateResult::NoReplanningNeeded));
    assert!(term.reset_counts.is_empty()); assert_eq!(term.evaluates, 0);
    assert_eq!(p.current_path, before);
    assert_eq!(p.node(8).tree_id, P::PRUNED_TREE_ID);
    assert_static_forest(&p, &[wall]);
}

#[test]
fn reset_repair_budget_still_returns_no_route_when_native_budget_expires() {
    let mut p = line(false); let wall = sphere(3., 0., 0.3);
    let mut term = ScanExpiredBudget::new(0);
    let result = p.update_new_static_obstacles_with_cursor(Box::new(checker(&[wall.clone()])),
        &term.counted_checker(&[wall.clone()]), Some(&mut term), S::new([0., 0.]), 0);
    assert!(matches!(result, SmartUpdateResult::ReplanningFailed));
    assert_eq!(term.reset_counts.len(), 1); assert_eq!(term.evaluates, 1);
    assert!(p.current_path.is_none());
    assert_eq!(p.node(3).tree_id, P::PRUNED_TREE_ID);
    assert_static_forest(&p, &[wall]);
}

#[test]
fn confirmed_static_removal_reactivates_only_isolated_now_valid_sample_ids() {
    let mut p = line(false); let kept = sphere(3., 0., 0.2); let removed = sphere(5., 0., 0.2);
    let walls = [kept.clone(), removed.clone()];
    apply(&mut p, &walls, &walls, [0., 0.]);
    assert_eq!(p.restore_valid_static_samples(), 0); // full checker still blocks
    p.set_static_validity_checker(Box::new(checker(&[kept.clone()])));
    let nodes_before = p.nodes.len(); let old_path = p.current_path.clone();
    let retained: Vec<_> = p.nodes.iter().enumerate().filter(|(_, n)| n.tree_id != P::PRUNED_TREE_ID)
        .map(|(id, n)| (id, n.parent, n.tree_id)).collect();
    let previous_id = p.tree_id_counter;
    assert_eq!(p.restore_valid_static_samples(), 1);
    assert_eq!(p.nodes.len(), nodes_before);
    assert_eq!(p.node(1).state, S::new([5., 0.])); // same historical sample ID
    assert!(p.node(1).tree_id > previous_id);
    assert!(p.node(1).parent.is_none() && p.node(1).children.is_empty());
    assert!(p.node(1).edge_cost.is_infinite() && p.node(1).cumulative_cost.is_infinite());
    assert_eq!(p.node(3).tree_id, P::PRUNED_TREE_ID); // retained obstacle wins
    for (id, parent, tree_id) in retained {
        assert_eq!(p.node(id).parent, parent); assert_eq!(p.node(id).tree_id, tree_id);
    }
    assert_eq!(p.current_path, old_path); // no route certificate or hidden rewire
    assert_eq!(p.restore_valid_static_samples(), 0);
    assert_static_forest(&p, &[kept]);
}

#[test]
fn goal_root_clear_uses_special_main_restore_not_generic_sample_restore() {
    let mut p = line(false); let wall = sphere(6., 0., 0.2);
    apply(&mut p, &[wall.clone()], &[wall], [0., 0.]);
    assert_eq!(p.node(0).tree_id, P::PRUNED_TREE_ID);
    p.set_static_validity_checker(Box::new(checker(&[])));
    assert_eq!(p.node(0).tree_id, P::MAIN_TREE_ID);
    assert_eq!(p.restore_valid_static_samples(), 0);
    assert!(p.node(0).parent.is_none() && p.node(0).children.is_empty());
    assert_static_forest(&p, &[]);
}

// Offline candidate regressions: historical static cuts are revalidated edges,
// not a second tree and not a blind replay of temporary dynamic pruning.
fn drain_history(p: &mut P) -> (usize, usize) {
    let mut checked=0;let mut restored=0;
    for _ in 0..200 {
        let r=p.maintain_static_clear_regions(1,1,1,Duration::from_secs(1));
        assert!(r.regions_examined<=1 && r.nodes_examined<=1 && r.connections<=1);
        checked+=r.historical_edges_examined;restored+=r.historical_edges_restored;
        if r.pending_nodes+r.pending_regions==0 {return (checked,restored);}
    }
    panic!("historical edge queue did not drain with one-work-unit budget");
}

#[test]
fn historical_parent_chain_restores_exact_sample_ids_and_original_edges() {
    let mut p=line(false);let before:Vec<_>=p.nodes.iter().map(|n|(n.state,n.parent)).collect();
    let wall=sphere(3.,0.,0.2);apply(&mut p,&[wall.clone()],&[wall],[0.,0.]);
    assert_eq!(p.static_cut_parents.get(&3),Some(&2));
    assert_eq!(p.static_cut_parents.get(&4),Some(&3));
    p.set_static_validity_checker(Box::new(checker(&[])));
    p.queue_static_clear_regions(&[(S::new([3.,0.]),0.2)]);
    let (_,restored)=drain_history(&mut p);assert_eq!(restored,2);
    assert_eq!(p.nodes.iter().map(|n|(n.state,n.parent)).collect::<Vec<_>>(),before);
    assert!(p.static_cut_parents.is_empty());assert!(p.static_cut_children.is_empty());
    assert_static_forest(&p,&[]);
}

#[test]
fn historical_edge_reuse_rejects_simultaneous_new_wall_between_free_endpoints() {
    let mut p=line(false);let old=sphere(3.,0.,0.2);
    apply(&mut p,&[old.clone()],&[old],[0.,0.]);
    let new=sphere(3.5,0.,0.2);
    apply(&mut p,&[new.clone()],&[new.clone()],[0.,0.]);
    p.queue_static_clear_regions(&[(S::new([3.,0.]),0.2)]);
    assert_eq!(drain_history(&mut p).1,0);
    assert!(p.static_validity_checker.is_state_valid(p.node(3).state()));
    assert_ne!(p.node(3).tree_id,P::MAIN_TREE_ID);
    assert_eq!(p.static_cut_parents.get(&3),Some(&2));
    assert_static_forest(&p,&[new]);
    p.set_static_validity_checker(Box::new(checker(&[])));
    // Put this child first so the assertion covers the historical fast path;
    // a MAIN neighbor processed first may already reconnect it generically.
    p.enqueue_static_clear_node(3);
    p.queue_static_clear_regions(&[(S::new([3.5,0.]),0.2)]);
    assert!(drain_history(&mut p).1>=1);assert_static_forest(&p,&[]);
    assert_eq!(p.node(3).parent,Some(2));
}

#[test]
fn historical_blocked_parent_later_activation_wakes_parked_child_without_spin() {
    let mut p=line(false);let child_wall=sphere(3.,0.,0.2);let parent_wall=sphere(4.,0.,0.2);
    apply(&mut p,&[child_wall.clone(),parent_wall.clone()],&[child_wall,parent_wall.clone()],[0.,0.]);
    p.set_static_validity_checker(Box::new(checker(&[parent_wall.clone()])));
    p.enqueue_static_clear_node(3);
    drain_history(&mut p);
    assert_eq!(p.static_cut_parents.get(&3),Some(&2));
    assert_eq!(p.node(2).tree_id,P::PRUNED_TREE_ID);
    assert!(p.static_clear_nodes.is_empty());
    for _ in 0..4 {
        let r=p.maintain_static_clear_regions(1,1,1,Duration::from_secs(1));
        assert_eq!(r.nodes_examined+r.connections,0,"blocked history must not spin");
    }
    p.set_static_validity_checker(Box::new(checker(&[])));
    // Only parent activation is scheduled here; reverse history must wake 3.
    p.enqueue_static_clear_node(2);
    assert!(drain_history(&mut p).1>=2);
    assert_eq!(p.node(3).parent,Some(2));assert_eq!(p.node(3).tree_id,P::MAIN_TREE_ID);
    assert_static_forest(&p,&[]);
}

#[test]
fn historical_direction_cycle_is_discarded_without_active_cycle() {
    let mut p=line(false);p.current_path=None;
    p.set_node_parent(1,None);p.node_mut(1).tree_id=1;p.node_mut(1).cumulative_cost=f32::INFINITY;
    p.propagate_info_to_children(1);
    p.set_node_parent(2,None);p.node_mut(2).tree_id=2;p.node_mut(2).cumulative_cost=f32::INFINITY;
    p.propagate_info_to_children(2);p.tree_id_counter=2;
    // Possible across separate prior rewires: references are not active edges.
    p.remember_static_cut_parent(1,2);p.remember_static_cut_parent(2,1);
    p.enqueue_static_clear_node(1);p.enqueue_static_clear_node(2);
    drain_history(&mut p);
    assert!(p.static_cut_parents.is_empty());assert!(p.static_cut_children.is_empty());
    assert_static_forest(&p,&[]);assert_eq!(p.nodes.len(),7);
}

#[test]
fn historical_edge_with_endpoints_outside_old_radius_recovers_on_interior_clear() {
    let states=[S::new([12.,0.]),S::new([0.,0.])];let mut nn=LinearNearestNeighbors::new();
    let mut nodes=Vec::new();for (id,s) in states.into_iter().enumerate() {
        nn.add(s,id);nodes.push(rrt_star::Node::new(s,if id==0{None}else{Some(0)},12.*id as f32));
    }
    let mut p=P::new_from_initial_rrt_star_tree(&nodes,nn,S::new([0.,0.]),S::new([12.,0.]),0.01,
        Box::new(checker(&[])),Box::new(UniformDistribution::new([(-1.,13.),(-1.,1.)])),
        9.,1.,1.5,10.,1.7,0.01,false,0.01);p.initial_plan_build_path();
    let wall=sphere(6.,0.,0.2);apply(&mut p,&[wall.clone()],&[wall],[0.,0.]);
    assert_eq!(p.node(1).parent,None);assert_eq!(p.static_cut_max_length(),12.);
    p.set_static_validity_checker(Box::new(checker(&[])));
    p.queue_static_clear_regions(&[(S::new([6.,0.]),0.2)]);
    assert_eq!(drain_history(&mut p).1,1);
    assert_eq!(p.node(1).parent,Some(0));assert_static_forest(&p,&[]);
}

#[test]
fn historical_recovery_never_rewires_existing_main_path_or_nodes() {
    let mut p=line(true);let wall=sphere(3.,0.,0.2);
    apply(&mut p,&[wall.clone()],&[wall],[0.,0.]);
    let path=p.current_path.clone();let parents:Vec<_>=p.nodes.iter().enumerate()
        .filter(|(_,n)|n.tree_id==P::MAIN_TREE_ID).map(|(id,n)|(id,n.parent)).collect();
    let count=p.nodes.len();p.set_static_validity_checker(Box::new(checker(&[])));
    p.queue_static_clear_regions(&[(S::new([3.,0.]),1.)]);drain_history(&mut p);
    assert_eq!(p.current_path,path);assert_eq!(p.nodes.len(),count);
    for (id,parent) in parents {assert_eq!(p.node(id).parent,parent);}
    assert_static_forest(&p,&[]);
}

#[test]
fn historical_cache_overwrite_removes_reverse_and_repeated_clear_is_bounded() {
    let mut p=line(false);let wall=sphere(3.,0.,0.2);
    for _ in 0..20 {
        apply(&mut p,&[wall.clone()],&[wall.clone()],[0.,0.]);
        assert!(p.static_cut_parents.len()<=p.nodes.len());
        assert_eq!(p.static_cut_children.values().map(|s|s.len()).sum::<usize>(),p.static_cut_parents.len());
        p.remember_static_cut_parent(3,1);
        assert!(!p.static_cut_children.get(&2).is_some_and(|s|s.contains(&3)));
        p.remember_static_cut_parent(3,2);
        assert!(!p.static_cut_children.get(&1).is_some_and(|s|s.contains(&3)));
        p.set_static_validity_checker(Box::new(checker(&[])));
        p.queue_static_clear_regions(&[(S::new([3.,0.]),1.)]);drain_history(&mut p);
        assert!(p.static_cut_parents.is_empty());assert!(p.static_cut_children.is_empty());
        assert_eq!(p.nodes.len(),7);assert_static_forest(&p,&[]);
    }
}

#[test]
fn historical_main_child_absorbs_old_parent_in_reverse_without_main_rewire() {
    let mut p=line(false);p.current_path=None;
    // Two isolated historical endpoints: child 3 has since reached MAIN via 0;
    // its old parent 2 remains disconnected. The edge must work in reverse.
    p.set_node_parent(2,None);p.node_mut(2).tree_id=1;p.node_mut(2).cumulative_cost=f32::INFINITY;
    p.propagate_info_to_children(2);p.tree_id_counter=1;
    p.make_root_of_subtree(3);p.set_node_parent(3,Some(0));
    p.node_mut(3).edge_cost=3.;p.node_mut(3).cumulative_cost=3.;p.node_mut(3).tree_id=0;
    // 2 was made a child of 3 during reroot; separate just that old-parent branch.
    p.set_node_parent(2,None);p.node_mut(2).tree_id=1;p.node_mut(2).cumulative_cost=f32::INFINITY;
    p.propagate_info_to_children(2);p.propagate_info_to_children(3);
    p.remember_static_cut_parent(3,2);assert_eq!(p.static_cut_parents.get(&3),Some(&2));
    let main_before:Vec<_>=p.nodes.iter().enumerate().filter(|(_,n)|n.tree_id==0).map(|(id,n)|(id,n.parent)).collect();
    p.enqueue_static_clear_node(3);let (_,restored)=drain_history(&mut p);
    assert_eq!(restored,1);assert_eq!(p.node(2).parent,Some(3));
    for (id,parent) in main_before {assert_eq!(p.node(id).parent,parent);}
    assert!(p.static_cut_parents.is_empty());assert_eq!(p.static_cut_max_length(),0.);
    assert_static_forest(&p,&[]);
}

#[test]
fn historical_long_edge_does_not_permanently_expand_later_clear_queries() {
    let mut p=line(false);let wall=sphere(3.,0.,0.2);
    apply(&mut p,&[wall.clone()],&[wall],[0.,0.]);
    p.remember_static_cut_parent(3,0);assert_eq!(p.static_cut_max_length(),3.);
    p.remember_static_cut_parent(3,2);assert_eq!(p.static_cut_max_length(),1.);
    p.set_static_validity_checker(Box::new(checker(&[])));
    p.queue_static_clear_regions(&[(S::new([3.,0.]),1.)]);drain_history(&mut p);
    assert_eq!(p.static_cut_max_length(),0.);assert!(p.static_cut_edge_lengths.is_empty());
}

#[test]
fn clear_priority_repairs_detached_samples_before_dense_main_backlog() {
    let mut p = line(false);
    let wall = sphere(3., 0., 0.2);
    apply(&mut p, &[wall.clone()], &[wall], [0., 0.]);
    assert_eq!(p.node(3).tree_id, P::PRUNED_TREE_ID);
    p.set_static_validity_checker(Box::new(checker(&[])));
    p.queue_static_clear_regions(&[(S::new([3.,0.]),0.5)]);
    let result = p.maintain_static_clear_regions(1,1,1,Duration::from_secs(1));
    assert_eq!(result.nodes_examined,1);
    assert_eq!(result.restored_nodes,1);
    assert_eq!(p.node(3).tree_id,P::MAIN_TREE_ID);
}

#[test]
fn clear_queue_keeps_main_endpoint_to_restore_component_outside_query_radius() {
    let mut nodes=Vec::new();let mut nn=LinearNearestNeighbors::new();
    for id in 0..3 {
        let state=S::new([id as f32*1.5,0.]);nn.add(state,id);
        nodes.push(rrt_star::Node::new(state,if id==0{None}else{Some(id-1)},id as f32*1.5));
    }
    let mut p=SMART::new_from_initial_rrt_star_tree(&nodes,nn,S::new([0.,0.]),S::new([0.,0.]),
        0.01,Box::new(checker(&[])),Box::new(UniformDistribution::new([(-5.,15.),(-5.,5.)])),
        9.,1.,1.5,10.,1.7,0.01,false,0.01);
    p.initial_plan_build_path();p.set_node_parent(2,None);p.tree_id_counter+=1;
    p.node_mut(2).tree_id=p.tree_id_counter;p.node_mut(2).edge_cost=f32::INFINITY;p.node_mut(2).cumulative_cost=f32::INFINITY;
    p.queue_static_clear_regions(&[(S::new([0.,0.]),0.)]);
    // Only MAIN 0/1 are queried (radius 1.7). Detached 2 is at x=3.
    let (_,connections)=drain_clear(&mut p);
    assert!(connections>=1);assert_eq!(p.node(2).tree_id,P::MAIN_TREE_ID);
    assert_eq!(p.node(2).parent,Some(1));assert_eq!(p.nodes.len(),3);
    assert_static_forest(&p,&[]);
}


fn drain_coverage(p: &mut P) -> usize {
    let mut added=0;
    for _ in 0..300 {
        drain_clear(p);
        let r=p.maintain_static_clear_coverage(16,2,100_000,Duration::from_secs(1));
        assert!(r.sampling_attempts<=16 && r.sampling_added_nodes<=2 && r.connections<=32);
        added+=r.sampling_added_nodes;
        if r.sampling_pending_regions==0 {return added;}
    }
    panic!("finite coverage queue did not drain");
}
fn missing_bridge_fixture() -> (P,usize,S) {
    let mut p=line(false);let center=S::new([3.0,1.5]);
    p.tree_id_counter+=1;
    let detached=p.add_node(Node::new(S::new([3.0,3.0]),None,f32::INFINITY,f32::INFINITY,p.tree_id_counter));
    assert!(p.get_feasible_neighbors_of_different_tree(p.node(detached)).is_empty());
    (p,detached,center)
}

#[test]
fn clear_coverage_bridges_initially_unsampled_gap_while_current_path_is_safe() {
    let (mut p,detached,center)=missing_bridge_fixture();
    let initial_wall=sphere(3.0,1.5,0.6);let retained_wall=sphere(4.4,1.5,0.4);
    p.set_static_validity_checker(Box::new(checker(&[initial_wall,retained_wall.clone()])));
    assert!(!p.static_validity_checker.is_state_valid(&center));
    assert!(p.nearest_neighbors.within_radius(&center,0.8).is_empty());
    let path=p.current_path.clone();let original_nodes=p.nodes.len();
    let parents:Vec<_>=p.nodes.iter().enumerate().filter(|(_,n)|n.tree_id==0).map(|(i,n)|(i,n.parent)).collect();
    p.set_static_validity_checker(Box::new(checker(&[retained_wall.clone()])));
    p.queue_static_clear_regions(&[(center,1.4)]);drain_clear(&mut p);
    assert_ne!(p.node(detached).tree_id,P::MAIN_TREE_ID);
    assert!(matches!(dynamic(&mut p,&[]),SmartUpdateResult::NoReplanningNeeded));
    assert_eq!(p.nodes.len(),original_nodes);
    assert!(drain_coverage(&mut p)>0);
    assert_eq!(p.node(detached).tree_id,P::MAIN_TREE_ID);
    assert_eq!(p.current_path,path);
    for (id,parent) in parents {assert_eq!(p.node(id).parent,parent);}
    assert_static_forest(&p,&[retained_wall]);
}

#[test]
fn repeated_clear_coverage_is_idempotent_and_keeps_original_ids() {
    let (mut p,detached,center)=missing_bridge_fixture();
    let positions:Vec<_>=p.nodes.iter().map(|n|n.state).collect();
    p.queue_static_clear_regions(&[(center,1.4)]);assert!(drain_coverage(&mut p)>0);
    let count=p.nodes.len();assert_eq!(p.node(detached).tree_id,P::MAIN_TREE_ID);
    for _ in 0..40 {
        p.queue_static_clear_regions(&[(center,1.4)]);
        assert_eq!(drain_coverage(&mut p),0);assert_eq!(p.nodes.len(),count);
    }
    for (id,state) in positions.iter().enumerate(){assert_eq!(p.node(id).state(),state);}
}

#[test]
fn coverage_waits_for_real_clear_and_never_crosses_remaining_wall() {
    let (mut p,detached,center)=missing_bridge_fixture();
    let wall=sphere(3.0,1.5,1.1);p.set_static_validity_checker(Box::new(checker(&[wall.clone()])));
    let count=p.nodes.len();p.queue_static_clear_regions(&[(center,0.4)]);
    assert_eq!(drain_coverage(&mut p),0);assert_eq!(p.nodes.len(),count);
    assert_ne!(p.node(detached).tree_id,P::MAIN_TREE_ID);
    assert_static_forest(&p,&[wall]);
}

#[test]
fn coverage_respects_zero_time_attempt_node_and_arena_caps() {
    let (mut p,_,center)=missing_bridge_fixture();p.queue_static_clear_regions(&[(center,1.4)]);
    drain_clear(&mut p);let count=p.nodes.len();
    assert_eq!(p.maintain_static_clear_coverage(16,2,100_000,Duration::ZERO).sampling_attempts,0);
    assert_eq!(p.maintain_static_clear_coverage(0,2,100_000,Duration::from_secs(1)).sampling_attempts,0);
    assert_eq!(p.maintain_static_clear_coverage(16,0,100_000,Duration::from_secs(1)).sampling_attempts,0);
    assert_eq!(p.maintain_static_clear_coverage(16,2,count,Duration::from_secs(1)).sampling_attempts,0);
    let r=p.maintain_static_clear_coverage(1,1,count+1,Duration::from_secs(1));
    assert_eq!(r.sampling_attempts,1);assert_eq!(r.sampling_added_nodes,1);assert_eq!(p.nodes.len(),count+1);
}

#[test]
fn coverage_event_queue_is_bounded_and_prior_existing_bridge_needs_no_sample() {
    let mut p=line(false);p.tree_id_counter+=1;
    let orphan=p.add_node(Node::new(S::new([3.0,1.5]),None,f32::INFINITY,f32::INFINITY,p.tree_id_counter));
    let count=p.nodes.len();p.queue_static_clear_regions(&[(S::new([3.0,0.75]),0.4)]);
    drain_clear(&mut p);assert_eq!(p.node(orphan).tree_id,P::MAIN_TREE_ID);
    assert_eq!(drain_coverage(&mut p),0);assert_eq!(p.nodes.len(),count);
    for i in 0..400 {p.queue_static_clear_regions(&[(S::new([i as f32,50.0]),1.0)]);}
    assert_eq!(p.static_clear_sampling_regions.len(),P::CLEAR_SAMPLING_MAX_REGIONS);
    assert_eq!(p.static_clear_sampling_keys.len(),P::CLEAR_SAMPLING_MAX_REGIONS);
    let r=p.maintain_static_clear_coverage(usize::MAX,usize::MAX,usize::MAX,Duration::from_secs(1));
    assert!(r.sampling_attempts<=P::CLEAR_SAMPLING_MAX_ATTEMPTS);
    assert!(r.sampling_added_nodes<=P::CLEAR_SAMPLING_MAX_ADDED);
}


#[test]
fn coverage_checks_free_tile_edges_even_when_center_stays_blocked() {
    let (mut p,detached,center)=missing_bridge_fixture();let wall=sphere(3.0,1.5,0.4);
    p.set_static_validity_checker(Box::new(checker(&[wall.clone()])));
    assert!(!p.static_validity_checker.is_state_valid(&center));
    let path=p.current_path.clone();p.queue_static_clear_regions(&[(center,1.6)]);
    assert!(drain_coverage(&mut p)>0);
    assert_eq!(p.node(detached).tree_id,P::MAIN_TREE_ID);assert_eq!(p.current_path,path);
    assert_static_forest(&p,&[wall]);
}

#[test]
fn newly_sampled_bridge_is_reused_after_obstacle_cut_and_clear_without_growth() {
    let (mut p,_,center)=missing_bridge_fixture();let original=p.nodes.len();
    p.queue_static_clear_regions(&[(center,1.4)]);assert!(drain_coverage(&mut p)>0);
    let count=p.nodes.len();let added:Vec<_>=(original..count).map(|id|(id,p.node(id).state)).collect();
    let wall=sphere(3.0,1.5,0.2);
    assert!(apply(&mut p,&[wall.clone()],&[wall],[0.0,0.0]).is_success());
    assert!(added.iter().any(|(id,_)|p.node(*id).tree_id==P::PRUNED_TREE_ID));
    p.set_static_validity_checker(Box::new(checker(&[])));
    p.queue_static_clear_regions(&[(center,1.4)]);assert_eq!(drain_coverage(&mut p),0);
    assert_eq!(p.nodes.len(),count);
    for (id,state) in added {assert_eq!(p.node(id).state,state);assert_eq!(p.node(id).tree_id,P::MAIN_TREE_ID);}
    assert_static_forest(&p,&[]);
}

#[test]
fn coverage_retries_initially_unreachable_center_after_real_edge_progress() {
    let mut p=line(false);let center=S::new([3.0,2.6]);
    assert!(p.nearest_neighbors.within_radius(&center,1.7).is_empty());
    let path=p.current_path.clone();p.queue_static_clear_regions(&[(center,2.0)]);
    assert!(drain_coverage(&mut p)>=2);
    assert!(!p.nearest_neighbors.within_radius(&center,0.001).is_empty(),
        "center needs a later finite pass after y=1.6 frontier is inserted");
    assert_eq!(p.current_path,path);
    assert_static_forest(&p,&[]);
    let count=p.nodes.len();p.queue_static_clear_regions(&[(center,2.0)]);
    assert_eq!(drain_coverage(&mut p),0);assert_eq!(p.nodes.len(),count);
}


#[test]
fn coverage_above_100k_arena_samples_without_rewiring_existing_main() {
    let (mut p,detached,center)=missing_bridge_fixture();
    let path=p.current_path.clone();
    // Real arena entries AND nearest-neighbor records, not a mocked count.
    // A distant valid MAIN chain cannot supply neighbors for the local gap.
    let mut parent=0;
    while p.nodes.len()<100_001 {
        let id=p.nodes.len();
        let state=S::new([1000.0+id as f32,1000.0]);
        let edge=p.node(parent).state().euclidean_distance(&state);
        let cost=p.node(parent).cumulative_cost+edge;
        let added=p.add_node(Node::new(state,Some(parent),edge,cost,P::MAIN_TREE_ID));
        p.node_mut(parent).children.push(added);parent=added;
    }
    let count=p.nodes.len();
    let original:Vec<_>=p.nodes.iter().map(|n|(n.state,n.parent,n.tree_id,n.edge_cost,n.cumulative_cost)).collect();
    p.queue_static_clear_regions(&[(center,1.4)]);
    // Explicit caller arena cap must still stop all work.
    assert_eq!(p.maintain_static_clear_coverage(1,1,count,Duration::from_secs(1)).sampling_attempts,0);
    let out=p.maintain_static_clear_coverage(1,1,250_000,Duration::from_secs(1));
    assert_eq!(out.sampling_attempts,1);
    assert_eq!(out.sampling_added_nodes,1);
    assert_eq!(p.nodes.len(),count+1);
    assert_eq!(p.node(detached).tree_id,P::MAIN_TREE_ID);
    assert_eq!(p.current_path,path);
    for (id,(state,parent,tree,edge,cost)) in original.into_iter().enumerate() {
        assert_eq!(p.node(id).state,state);
        if tree==P::MAIN_TREE_ID {
            assert_eq!(p.node(id).parent,parent);
            assert_eq!(p.node(id).tree_id,tree);
            assert_eq!(p.node(id).edge_cost,edge);
            assert_eq!(p.node(id).cumulative_cost,cost);
        }
    }
    assert!(p.static_validity_checker.is_state_valid(p.node(count).state()));
    let parent=p.node(count).parent.unwrap();
    assert!(p.static_validity_checker.is_edge_valid(p.node(count).state(),p.node(parent).state()));
    assert_eq!(P::CLEAR_SAMPLING_MAX_ARENA,250_000);
    assert!(p.debug_check_finalized_forest());
}
