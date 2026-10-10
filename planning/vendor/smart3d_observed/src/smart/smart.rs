use std::collections::{BTreeMap, BTreeSet, HashMap, VecDeque};
use std::time::{Duration, Instant};

use rand::Rng;

use crate::obstacles::SphericalObstacle;
#[cfg(test)]
use crate::obstacles::StaticSphericalObstacle;
use crate::rrt::rrt_star;
use crate::rrt::{
    NearestNeighbors, RealVectorState, SamplingDistribution, TerminationCondition, ValidityChecker,
};
use crate::smart::obstacle::HazardZone;
use crate::smart::{
    node::convert_rrt_star_nodes_to_smart_nodes, Node, ReplanningTriggerCondition,
    SmartUpdateResult, SphericalObstacleWithOhz,
};
use crate::util::ordered_float::OrderedFloat;
use num_traits::Float;

/// Self-Morphing Adaptive Replanning Tree (SMART) algorithm.
pub struct SMART<F: Float, const N: usize, NN> {
    // Problem definition
    goal: RealVectorState<F, N>, // The goal state
    goal_tolerance: F,           // The goal tolerance
    static_validity_checker: Box<dyn ValidityChecker<F, N>>, // The static validity checker

    // SMART planning parameters
    sampling_distribution: Box<dyn SamplingDistribution<F, N>>,
    lrz_radius: F,
    lsr_initial_radius: F, // The initial radius of the LSR (Local Search Region)
    lsr_expansion_factor: F, // The factor by which the LSR expands
    lsr_max_radius: F,     // The maximum radius of the LSR
    hot_node_neighborhood_radius: F,
    path_node_radius: F, // When we look for a path, we search for starting nodes within this radius
    replan_when_path_is_safe_but_robot_inside_ohz: bool, // Whether to replan when the path is safe but the robot is inside an OHZ

    // OHZ margin is a special parameter for when the robot is inside an OHZ.
    // When the robot is inside an OHZ, the OHZ is shrunk to the distance between the robot and obstacle.
    // However, for planning purposes, we need to further shrink the OHZ by a small margin such that the robot is not inside the OHZ.
    // This is done to avoid the robot being inside the OHZ. If the robot is inside the OHZ, all validity checks for all edges to/from the robot will fail.
    // This parameter is used regardless of the value of `replan_when_path_is_safe_but_robot_inside_ohz`.
    ohz_shrinkage_margin: F,

    // Keep track of current phase
    initial_planning_phase: bool, // Whether we are in the initial planning phase

    // SMART tree state
    nodes: Vec<Node<F, N>>, // The nodes in the tree
    nearest_neighbors: NN,

    // SMART inputs (provided by user during each update)
    cpr_obstacles: Vec<HazardZone<F, N>>, // The obstacles with which intersect the LRZ (derived from dynamic obstacles).
    // We use a vector of static obstacles, but these are actually one snapshot of the dynamic obstacles at the given timestep.
    max_cpr_radius: F, // The maximum radius for which dynamic obstacle collision checking is needed (derived from cpr_obstacles).
    // max_cpr_radius is the maximum distance between the current state and the furthest point of any CPR obstacle.
    current_state: RealVectorState<F, N>, // The current state of the robot
    robot_inside_ohzs: Vec<HazardZone<F, N>>, // The obstacles with which the robot is inside the OHZ (derived from dynamic obstacles).

    // SMART path (updated during each update)
    current_path: Option<Vec<usize>>,

    // Temporarily variables created and consumed during each update
    tree_id_counter: i32,
    disjoint_tree_roots: Vec<(usize, Option<usize>)>, // (node_id, old_parent_id)
    pruned_nodes: Vec<(usize, Option<usize>)>,        // (node_id, old_parent_id)
    optimization_start_nodes: Vec<usize>,
    replanning_triggered_by: Option<ReplanningTriggerCondition>,
    // Explicit map-clearing maintenance, independent of robot-local CPR repair.
    // Regions and node IDs are deduplicated; no full-arena scan on clear ticks.
    static_clear_regions: VecDeque<(Vec<u64>, RealVectorState<F, N>, F)>,
    static_clear_priority_streak: usize,
    // Best-effort sparse coverage work for explicit clears, bounded separately
    // from historical sample recovery. Each region has a fixed finite stencil.
    static_clear_sampling_regions: VecDeque<(Vec<u64>, RealVectorState<F, N>, F, usize, usize, bool)>,
    static_clear_sampling_keys: BTreeSet<Vec<u64>>,
    static_clear_region_keys: BTreeSet<Vec<u64>>,
    static_clear_nodes: VecDeque<usize>,
    static_clear_node_ids: BTreeSet<usize>,
    // Historical static cuts are candidates, never active/displayed tree edges.
    // One most recent real cut parent per child; node IDs remain in this arena.
    static_cut_parents: HashMap<usize, usize>,
    static_cut_children: HashMap<usize, BTreeSet<usize>>,
    static_cut_edge_lengths: BTreeMap<OrderedFloat<F>, usize>,
}

#[derive(Default, Debug)]
pub struct StaticClearMaintenance {
    pub regions_examined: usize,
    pub nodes_examined: usize,
    pub restored_nodes: usize,
    pub connections: usize,
    pub pending_regions: usize,
    pub pending_nodes: usize,
    pub historical_edges_examined: usize,
    pub historical_edges_restored: usize,
    pub retained_cut_edges: usize,
    pub historical_max_edge_length_m: f64,
    pub region_query_time_us: u64,
    pub sampling_attempts: usize,
    pub sampling_added_nodes: usize,
    pub sampling_pending_regions: usize,
}

#[derive(Clone)]
struct HotNode<F> {
    node_id: usize,
    feasible_neighbors_of_different_tree: Vec<usize>,
    utility: F,
}

impl<F: Float + std::fmt::Debug, const N: usize, NN: NearestNeighbors<F, N>> SMART<F, N, NN> {
    const MAIN_TREE_ID: i32 = 0; // The ID of the main tree
    const PRUNED_TREE_ID: i32 = -1;
    const CLEAR_SAMPLING_MAX_ATTEMPTS: usize = 64;
    const CLEAR_SAMPLING_MAX_ADDED: usize = 8;
    const CLEAR_SAMPLING_MAX_ARENA: usize = 250_000;
    const CLEAR_SAMPLING_MAX_REGIONS: usize = 256; // The ID of a pruned node

    /// Creates a new SMART planner from an RRT star tree.
    pub fn new_from_initial_rrt_star_tree(
        rrt_star_nodes: &Vec<rrt_star::Node<F, N>>,
        rrt_star_nearest_neighbors: NN,
        start: RealVectorState<F, N>,
        goal: RealVectorState<F, N>,
        goal_tolerance: F,
        static_validity_checker: Box<dyn ValidityChecker<F, N>>,
        sampling_distribution: Box<dyn SamplingDistribution<F, N>>,
        lrz_radius: F,
        lsr_initial_radius: F,
        lsr_expansion_factor: F,
        lsr_max_radius: F,
        hot_node_neighborhood_radius: F,
        path_node_radius: F,
        replan_when_path_is_safe_but_robot_inside_ohz: bool,
        ohz_shrinkage_margin: F,
    ) -> Self {
        // Create the SMART planner
        let mut smart = Self {
            goal,
            goal_tolerance,
            static_validity_checker,
            sampling_distribution,
            lrz_radius,
            lsr_initial_radius,
            lsr_expansion_factor,
            lsr_max_radius,
            hot_node_neighborhood_radius,
            path_node_radius,
            replan_when_path_is_safe_but_robot_inside_ohz,
            ohz_shrinkage_margin,
            initial_planning_phase: true,
            nodes: convert_rrt_star_nodes_to_smart_nodes(rrt_star_nodes),
            nearest_neighbors: rrt_star_nearest_neighbors,
            cpr_obstacles: Vec::new(),
            max_cpr_radius: F::zero(),
            current_state: start.clone(),
            robot_inside_ohzs: Vec::new(),
            current_path: None,
            tree_id_counter: 0,
            disjoint_tree_roots: Vec::new(),
            pruned_nodes: Vec::new(),
            optimization_start_nodes: Vec::new(),
            replanning_triggered_by: None,
            static_clear_regions: VecDeque::new(),
            static_clear_priority_streak: 0,
            static_clear_sampling_regions: VecDeque::new(),
            static_clear_sampling_keys: BTreeSet::new(),
            static_clear_region_keys: BTreeSet::new(),
            static_clear_nodes: VecDeque::new(),
            static_clear_node_ids: BTreeSet::new(),
            static_cut_parents: HashMap::new(),
            static_cut_children: HashMap::new(),
            static_cut_edge_lengths: BTreeMap::new(),
        };
        // RRT* rewiring changes a parent's cumulative cost without propagating
        // that change to its descendants. Conversion already derives the real
        // edge lengths; reconcile cumulative costs on those same retained edges
        // before SMART optimization compares them. No topology is changed.
        if !smart.nodes.is_empty() {
            smart.node_mut(0).cumulative_cost = F::zero();
            smart.propagate_info_to_children(0);
        }
        smart
    }

    /// Optimize the full tree from the initial plan.
    pub fn initial_plan_optimize_full_tree(&mut self) {
        assert!(
            self.initial_planning_phase,
            "Function only valid for initial planning"
        );
        self.rewire_cascade(0);
    }

    pub fn initial_plan_add_node_at_start(&mut self) {
        assert!(
            self.initial_planning_phase,
            "Function only valid for initial planning"
        );
        let new_node_state = self.current_state.clone();
        self.add_new_node_at_state_and_connect(new_node_state);
        self.tree_optimization();
    }

    pub fn initial_plan_build_path(&mut self) {
        assert!(
            self.initial_planning_phase,
            "Function only valid for initial planning"
        );
        debug_assert!(self.debug_check_all_nodes_in_main_tree());
        debug_assert!(self.debug_check_tree_consistency());
        debug_assert!(self.debug_check_costs());

        self.current_path = self.build_planned_path();
        self.initial_planning_phase = false; // Set the initial planning phase to false
    }

    /// Update the SMART planner and replan if necessary.
    ///
    /// Parameters:
    /// - `current_state`: The current state.
    /// - `path_index`: The index of the next state in the current path.
    /// - `dynamic_obstacles`: The dynamic obstacles with OHZ.
    /// - `termination`: The termination condition for the update.
    ///                  The termination condition only applies to the time taken for pruning, reconnection and sampling phases.
    ///                  Tree optimization and path building will be performed after the termination condition is met.
    pub fn update<TC>(
        &mut self,
        current_state: RealVectorState<F, N>,
        path_index: usize,
        dynamic_obstacles: &Vec<SphericalObstacleWithOhz<F, N>>,
        mut termination_condition: Option<&mut TC>,
    ) -> SmartUpdateResult
    where
        TC: TerminationCondition,
    {
        assert!(
            !self.initial_planning_phase,
            "Function only valid for non-initial planning"
        );

        // Update the current state
        self.current_state = current_state;

        // Check if current state is in collision with static obstacles
        if !self
            .static_validity_checker
            .is_state_valid(&self.current_state)
            || !self.static_validity_checker.is_state_valid(&self.goal)
        {
            self.current_path = None;
            return SmartUpdateResult::InCollision;
        }

        // Check if current state is in collision with dynamic obstacles or in OHZ of any of them
        for obs in dynamic_obstacles {
            if obs.obstacle().contains(&self.current_state) {
                return SmartUpdateResult::InCollision;
            }
        }

        // Build the CPR from the dynamic obstacles
        self.build_cpr(dynamic_obstacles);

        // Check if the current path is safe
        let (path_safe, unsafe_node_id) = self.is_path_safe_within_lrz(path_index);
        let hotnode_search_center;
        if !path_safe || !self.is_remaining_path_static_valid(path_index) {
            // The path is not safe, so we need to replan
            self.replanning_triggered_by = Some(ReplanningTriggerCondition::PathUnsafe);
            // center of the LSR is the position of the first pruned node on the path,
            // or the robot position if no unsafe node was identified (e.g. current_path is None)
            hotnode_search_center = if let Some(unsafe_node_id) = unsafe_node_id {
                Some(self.node(unsafe_node_id).state().clone())
            } else {
                Some(self.current_state.clone())
            };
        } else if self.replan_when_path_is_safe_but_robot_inside_ohz
            && !self.robot_inside_ohzs.is_empty()
        {
            // The robot is inside an OHZ, so we need to replan
            self.replanning_triggered_by = Some(ReplanningTriggerCondition::RobotInOHZ);
            hotnode_search_center = Some(self.current_state.clone());
        } else {
            // The path is safe and the robot is not inside any OHZ, so we can continue
            return SmartUpdateResult::NoReplanningNeeded;
        }

        self.replanning_triggered_by = Some(ReplanningTriggerCondition::PathUnsafe);

        // Clear the previous replanning results
        // Retained static forest components keep their IDs across repair calls.
        assert!(self.disjoint_tree_roots.is_empty());
        assert!(self.pruned_nodes.is_empty());
        assert!(self.optimization_start_nodes.is_empty());
        debug_assert!(self.debug_check_finalized_forest());
        debug_assert!(self.debug_check_tree_consistency());

        self.prune_tree(); // Prune the tree based on the CPR

        debug_assert!(self.debug_check_tree_consistency()); // Check that the tree is consistent after pruning
        debug_assert!(self.debug_check_pruning()); // Check that the pruning was done correctly
        debug_assert!(self.debug_check_costs()); // Check that the costs are correct after pruning

        // Begin replanning with SMART
        self.current_path = self.build_planned_path(); // Try to build a new planned path

        while self.current_path.is_none() {
            // Check if the termination condition is met
            if let Some(termination_condition) = termination_condition.as_mut() {
                if termination_condition.evaluate() {
                    self.tree_optimization(); // Optimize the tree before exiting
                    self.current_path = self.build_planned_path(); // Try to build a new planned path after optimization
                    self.form_single_tree(); // Form a single tree
                    debug_assert!(self.debug_check_tree_consistency()); // Check that the tree is consistent before exiting
                    debug_assert!(self.debug_check_costs()); // Check that the costs are correct before exiting
                    return SmartUpdateResult::ReplanningFailed;
                }
            }

            let best_hot_node = self.find_hot_node_with_expanding_lsr(
                hotnode_search_center.as_ref().unwrap());

            if best_hot_node.is_some() {
                let best_hot_node = best_hot_node.unwrap();
                self.reconnect_best_hot_node(&best_hot_node);
            } else {
                // sampling tree repair will sample new nodes to add to the tree
                self.sampling_tree_repair_iteration();
            }

            debug_assert!(self.debug_check_costs());
            debug_assert!(self.debug_check_non_pruned_nodes_are_safe());
            debug_assert!(self.debug_check_tree_consistency());

            self.current_path = self.build_planned_path(); // Search for a new path within the tree
        }

        self.tree_optimization(); // Optimize the tree after replanning
        debug_assert!(self.debug_check_tree_consistency()); // Check that the tree is consistent after optimization

        self.current_path = self.build_planned_path(); // Try to build a new planned path after optimization

        self.form_single_tree(); // Form a single tree
        debug_assert!(self.debug_check_finalized_forest());
        debug_assert!(self.debug_check_tree_consistency());

        SmartUpdateResult::ReplanningSuccessful
    }

    /// Update the static obstacles (e.g., when new walls are discovered by LiDAR).
    /// This performs incremental tree repair instead of rebuilding the entire tree.
    ///
    /// Steps:
    /// 1. Replace the static validity checker
    /// 2. Prune nodes inside new walls
    /// 3. Break edges crossing new walls
    /// 4. Create disjoint subtrees
    /// 5. Hot-node repair to reconnect
    /// 6. Optimize and rebuild path
    ///
    /// Parameters:
    /// - `new_static_validity_checker`: The updated validity checker including new walls
    /// - `termination_condition`: Optional repair budget. Reset immediately
    ///   before hot-node/sampling repair, after mandatory full-tree pruning and
    ///   the first path lookup; pruning and finalization are not budgeted here.
    pub fn update_static_obstacles<TC>(
        &mut self,
        new_static_validity_checker: Box<dyn ValidityChecker<F, N>>,
        mut termination_condition: Option<&mut TC>,
        current_robot_state: RealVectorState<F, N>,
    ) -> SmartUpdateResult
    where
        TC: TerminationCondition,
    {
        assert!(
            !self.initial_planning_phase,
            "Function only valid for non-initial planning"
        );

        // Update the current state so build_planned_path() searches from the robot's actual position
        self.current_state = current_robot_state;

        // Replace the static validity checker
        self.set_static_validity_checker(new_static_validity_checker);

        // Clear CPR state from any previous dynamic obstacle update
        // During static-only updates, no dynamic obstacles are involved
        self.cpr_obstacles.clear();
        self.max_cpr_radius = F::zero();
        self.robot_inside_ohzs.clear();

        // Clear previous replanning state
        // Retained static forest components keep their IDs across repair calls.
        self.replanning_triggered_by = Some(ReplanningTriggerCondition::PathUnsafe);
        assert!(self.disjoint_tree_roots.is_empty());
        assert!(self.pruned_nodes.is_empty());
        assert!(self.optimization_start_nodes.is_empty());

        // Phase 1: Prune nodes inside new walls
        let mut pruned_nodes = Vec::new();
        for node_id in 0..self.nodes.len() {
            if self.node(node_id).tree_id == Self::PRUNED_TREE_ID {
                continue;
            }
            if !self
                .static_validity_checker
                .is_state_valid(self.node(node_id).state())
            {
                // Node is inside a new wall
                self.node_mut(node_id).tree_id = Self::PRUNED_TREE_ID;
                let old_parent_id = self.node(node_id).parent;
                self.set_node_parent(node_id, None);
                self.node_mut(node_id).cumulative_cost = F::infinity();
                pruned_nodes.push((node_id, old_parent_id));
            }
        }

        // Phase 2: Create disjoint subtrees from children of pruned nodes
        let mut disjoint_tree_roots = Vec::new();
        for &(node_id, _) in &pruned_nodes {
            let children = self.node(node_id).children.clone();
            for child_id in children {
                if self.node(child_id).tree_id == Self::PRUNED_TREE_ID {
                    continue;
                }
                self.tree_id_counter += 1;
                let new_tree_id = self.tree_id_counter;
                let old_parent_id = self.node(child_id).parent;
                self.set_node_parent(child_id, None);
                let child = self.node_mut(child_id);
                child.cumulative_cost = F::infinity();
                child.tree_id = new_tree_id;
                self.propagate_info_to_children(child_id);
                disjoint_tree_roots.push((child_id, old_parent_id));
            }
        }

        // Phase 3: Break edges that cross new walls (for all alive nodes)
        for node_id in 0..self.nodes.len() {
            if self.node(node_id).tree_id == Self::PRUNED_TREE_ID {
                continue;
            }
            if let Some(parent_id) = self.node(node_id).parent {
                if self.node(parent_id).tree_id == Self::PRUNED_TREE_ID {
                    continue; // parent already pruned, child already handled above
                }
                if !self
                    .static_validity_checker
                    .is_edge_valid(self.node(parent_id).state(), self.node(node_id).state())
                {
                    let old_parent_id = Some(parent_id);
                    self.set_node_parent(node_id, None);
                    self.tree_id_counter += 1;
                    self.node_mut(node_id).tree_id = self.tree_id_counter;
                    self.node_mut(node_id).cumulative_cost = F::infinity();
                    self.propagate_info_to_children(node_id);
                    disjoint_tree_roots.push((node_id, old_parent_id));
                }
            }
        }

        self.pruned_nodes = pruned_nodes;
        self.disjoint_tree_roots = disjoint_tree_roots;

        let num_pruned = self.pruned_nodes.len();
        let num_disjoint = self.disjoint_tree_roots.len();

        // New static evidence is committed even when it covers the robot/goal.
        if !self.static_validity_checker.is_state_valid(&self.current_state)
            || !self.static_validity_checker.is_state_valid(&self.goal)
        {
            self.current_path = None;
            self.form_single_tree_after_static_update();
            return SmartUpdateResult::InCollision;
        }
        // A previous failed repair still needs the native repair loop.
        if num_pruned == 0 && num_disjoint == 0 && self.is_remaining_path_static_valid(0) {
            return SmartUpdateResult::NoReplanningNeeded;
        }

        // Phase 4: Hot-node repair
        let hotnode_search_center = self.current_state.clone();
        self.current_path = self.build_planned_path();
        if self.current_path.is_none() {
            if let Some(tc) = termination_condition.as_mut() {
                tc.reset();
            }
        }

        while self.current_path.is_none() {
            if let Some(tc) = termination_condition.as_mut() {
                if tc.evaluate() {
                    self.tree_optimization();
                    self.current_path = self.build_planned_path();
                    return self.finish_static_repair();
                }
            }

            let best_hot_node = self.find_hot_node_with_expanding_lsr(&hotnode_search_center);

            if best_hot_node.is_some() {
                self.reconnect_best_hot_node(&best_hot_node.unwrap());
            } else {
                self.sampling_tree_repair_iteration();
            }

            self.current_path = self.build_planned_path();
        }

        // Phase 5: Optimize and finalize
        self.tree_optimization();
        self.current_path = self.build_planned_path();
        self.finish_static_repair()
    }

    /// Incremental tree repair checking only unvalidated walls.
    ///
    /// Same SMART repair flow as `update_static_obstacles()`, but Phase 1 (prune
    /// nodes) and Phase 3 (break edges) only check against the walls the tree
    /// hasn't been validated against yet (accumulated since last build/repair).
    ///
    /// Phase 4 (hot-node reconnection) and Phase 5 (path building) still use
    /// the full validity checker (all walls), ensuring new connections and the
    /// final path don't cross any wall, old or new.
    ///
    /// Parameters:
    /// - `new_static_validity_checker`: Checker with ALL known walls (replaces old)
    /// - `unvalidated_walls_checker`: Checker with only walls accumulated since last validation
    /// - `termination_condition`: Optional repair budget, reset after mandatory
    ///   delta pruning and the first path lookup if hot-node/sampling repair is
    ///   needed. Pruning and finalization are not budgeted here.
    /// - `current_robot_state`: Robot's current position
    pub fn update_new_static_obstacles<TC>(
        &mut self,
        new_static_validity_checker: Box<dyn ValidityChecker<F, N>>,
        unvalidated_walls_checker: &dyn ValidityChecker<F, N>,
        termination_condition: Option<&mut TC>,
        current_robot_state: RealVectorState<F, N>,
    ) -> SmartUpdateResult
    where
        TC: TerminationCondition,
    {
        self.update_new_static_obstacles_with_cursor(new_static_validity_checker,
            unvalidated_walls_checker, termination_condition, current_robot_state, 0)
    }

    /// Static increments must also invalidate a newly blocked measured-position
    /// connector, which need not be an arena edge. The cursor names the next
    /// actual execution waypoint; already traversed path segments are omitted.
    /// A replanned result starts the returned path at index zero, even when its
    /// coordinates happen to equal the previous path.
    pub fn update_new_static_obstacles_with_cursor<TC>(
        &mut self,
        new_static_validity_checker: Box<dyn ValidityChecker<F, N>>,
        unvalidated_walls_checker: &dyn ValidityChecker<F, N>,
        mut termination_condition: Option<&mut TC>,
        current_robot_state: RealVectorState<F, N>,
        current_path_index: usize,
    ) -> SmartUpdateResult
    where
        TC: TerminationCondition,
    {
        assert!(
            !self.initial_planning_phase,
            "Function only valid for non-initial planning"
        );

        self.current_state = current_robot_state;

        // Replace with full validity checker (all walls)
        self.set_static_validity_checker(new_static_validity_checker);

        // Clear CPR state
        self.cpr_obstacles.clear();
        self.max_cpr_radius = F::zero();
        self.robot_inside_ohzs.clear();

        // Clear replanning state
        // Retained static forest components keep their IDs across repair calls.
        self.replanning_triggered_by = Some(ReplanningTriggerCondition::PathUnsafe);
        assert!(self.disjoint_tree_roots.is_empty());
        assert!(self.pruned_nodes.is_empty());
        assert!(self.optimization_start_nodes.is_empty());

        // Phase 1: Prune nodes inside UNVALIDATED walls
        let mut pruned_nodes = Vec::new();
        for node_id in 0..self.nodes.len() {
            if self.node(node_id).tree_id == Self::PRUNED_TREE_ID {
                continue;
            }
            if !unvalidated_walls_checker.is_state_valid(self.node(node_id).state()) {
                self.node_mut(node_id).tree_id = Self::PRUNED_TREE_ID;
                let old_parent_id = self.node(node_id).parent;
                self.set_node_parent(node_id, None);
                self.node_mut(node_id).cumulative_cost = F::infinity();
                pruned_nodes.push((node_id, old_parent_id));
            }
        }

        // Phase 2: Create disjoint subtrees from children of pruned nodes
        let mut disjoint_tree_roots = Vec::new();
        for &(node_id, _) in &pruned_nodes {
            let children = self.node(node_id).children.clone();
            for child_id in children {
                if self.node(child_id).tree_id == Self::PRUNED_TREE_ID {
                    continue;
                }
                self.tree_id_counter += 1;
                let new_tree_id = self.tree_id_counter;
                let old_parent_id = self.node(child_id).parent;
                self.set_node_parent(child_id, None);
                let child = self.node_mut(child_id);
                child.cumulative_cost = F::infinity();
                child.tree_id = new_tree_id;
                self.propagate_info_to_children(child_id);
                disjoint_tree_roots.push((child_id, old_parent_id));
            }
        }

        // Phase 3: Break edges that cross UNVALIDATED walls
        for node_id in 0..self.nodes.len() {
            if self.node(node_id).tree_id == Self::PRUNED_TREE_ID {
                continue;
            }
            if let Some(parent_id) = self.node(node_id).parent {
                if self.node(parent_id).tree_id == Self::PRUNED_TREE_ID {
                    continue;
                }
                // Only check against unvalidated walls
                if !unvalidated_walls_checker
                    .is_edge_valid(self.node(parent_id).state(), self.node(node_id).state())
                {
                    let old_parent_id = Some(parent_id);
                    self.set_node_parent(node_id, None);
                    self.tree_id_counter += 1;
                    self.node_mut(node_id).tree_id = self.tree_id_counter;
                    self.node_mut(node_id).cumulative_cost = F::infinity();
                    self.propagate_info_to_children(node_id);
                    disjoint_tree_roots.push((node_id, old_parent_id));
                }
            }
        }

        self.pruned_nodes = pruned_nodes;
        self.disjoint_tree_roots = disjoint_tree_roots;

        // New static evidence is committed even when it covers the robot/goal.
        if !self.static_validity_checker.is_state_valid(&self.current_state)
            || !self.static_validity_checker.is_state_valid(&self.goal)
        {
            self.current_path = None;
            self.form_single_tree_after_static_update();
            return SmartUpdateResult::InCollision;
        }
        if self.is_remaining_path_static_valid(current_path_index) {
            // New evidence may cut only off-path branches. Finalize those cuts
            // without rebuilding the still-certified ordered path or resetting
            // its caller-owned cursor. The surviving MAIN path is unchanged.
            if !self.pruned_nodes.is_empty() || !self.disjoint_tree_roots.is_empty() {
                self.form_single_tree_after_static_update();
            }
            return SmartUpdateResult::NoReplanningNeeded;
        }

        // Phase 4: Hot-node repair (uses FULL validity checker for reconnection)
        let hotnode_search_center = self.current_state.clone();
        self.current_path = self.build_planned_path();
        if self.current_path.is_none() {
            if let Some(tc) = termination_condition.as_mut() {
                tc.reset();
            }
        }

        while self.current_path.is_none() {
            if let Some(tc) = termination_condition.as_mut() {
                if tc.evaluate() {
                    self.tree_optimization();
                    self.current_path = self.build_planned_path();
                    return self.finish_static_repair();
                }
            }

            let best_hot_node = self.find_hot_node_with_expanding_lsr(&hotnode_search_center);

            if best_hot_node.is_some() {
                self.reconnect_best_hot_node(&best_hot_node.unwrap());
            } else {
                self.sampling_tree_repair_iteration();
            }

            self.current_path = self.build_planned_path();
        }

        // Phase 5: Optimize and finalize
        self.tree_optimization();
        self.current_path = self.build_planned_path();
        self.finish_static_repair()
    }

    /// Replace the static validity checker without scanning the tree. New blocked
    /// space must still be applied through an incremental/static update before
    /// using the retained tree. Removing blocked space requires no bulk restore:
    /// only an isolated, now-valid goal root is reinstated for native reconnection.
    pub fn set_static_validity_checker(
        &mut self,
        new_static_validity_checker: Box<dyn ValidityChecker<F, N>>,
    ) {
        self.static_validity_checker = new_static_validity_checker;
        if let Some(goal_root) = self.nodes.first() {
            if goal_root.tree_id == Self::PRUNED_TREE_ID
                && goal_root.parent.is_none()
                && goal_root.children.is_empty()
                && self.static_validity_checker.is_state_valid(goal_root.state())
                && self.static_validity_checker.is_state_valid(&self.goal)
            {
                let root = self.node_mut(0);
                root.tree_id = Self::MAIN_TREE_ID;
                root.edge_cost = F::zero();
                root.cumulative_cost = F::zero();
                self.wake_static_cut_children(0);
            }
        }
    }

    /// Reactivate isolated historical samples after confirmed static-obstacle
    /// removal. Install the current full checker and apply any simultaneous new
    /// obstacles first. This does not restore old parents, certify a path, or
    /// run a search; the next native update may reconnect these forest roots.
    /// Call only on an explicit removal event, not on every observation cycle.
    pub fn restore_valid_static_samples(&mut self) -> usize {
        let mut restored = 0;
        // Goal node zero has its own MAIN-root restoration in the setter.
        for id in 1..self.nodes.len() {
            let node = self.node(id);
            if node.tree_id == Self::PRUNED_TREE_ID
                && node.parent.is_none() && node.children.is_empty()
                && self.static_validity_checker.is_state_valid(node.state())
            {
                self.tree_id_counter += 1;
                let tree_id = self.tree_id_counter;
                let node = self.node_mut(id);
                node.tree_id = tree_id;
                node.edge_cost = F::infinity();
                node.cumulative_cost = F::infinity();
                restored += 1;
                self.wake_static_cut_children(id);
            }
        }
        restored
    }

    fn forget_static_cut_parent(&mut self, child: usize) {
        if let Some(parent) = self.static_cut_parents.remove(&child) {
            let length = OrderedFloat::from(self.node(child).state().euclidean_distance(self.node(parent).state()));
            if let Some(count) = self.static_cut_edge_lengths.get_mut(&length) {
                *count -= 1;
                if *count == 0 { self.static_cut_edge_lengths.remove(&length); }
            }
            if let Some(children) = self.static_cut_children.get_mut(&parent) {
                children.remove(&child);
                if children.is_empty() { self.static_cut_children.remove(&parent); }
            }
        }
    }

    fn remember_static_cut_parent(&mut self, child: usize, parent: usize) {
        if child == parent || self.node(child).parent == Some(parent)
            || (self.node(child).tree_id != Self::PRUNED_TREE_ID
                && self.node(child).tree_id == self.node(parent).tree_id) {
            self.forget_static_cut_parent(child);
            return;
        }
        self.forget_static_cut_parent(child);
        self.static_cut_parents.insert(child, parent);
        self.static_cut_children.entry(parent).or_default().insert(child);
        // An edge can cross a clear region with neither endpoint inside it.
        // This conservative expansion also covers historical edges longer than
        // the current neighbor radius, without scanning the arena/cut table.
        let length = self.node(child).state().euclidean_distance(self.node(parent).state());
        *self.static_cut_edge_lengths.entry(OrderedFloat::from(length)).or_default() += 1;
    }

    fn static_cut_max_length(&self) -> F {
        self.static_cut_edge_lengths.last_key_value().map(|(length,_)|length.0).unwrap_or(F::zero())
    }

    fn enqueue_static_clear_node(&mut self, id: usize) {
        if self.static_clear_node_ids.insert(id) { self.static_clear_nodes.push_back(id); }
    }

    fn wake_static_cut_children(&mut self, parent: usize) {
        let children: Vec<_> = self.static_cut_children.get(&parent)
            .map(|ids| ids.iter().copied().collect()).unwrap_or_default();
        for child in children { self.enqueue_static_clear_node(child); }
    }

    // Revalidate the actual cut edge before any nearest-neighbor search. Never
    // rewire existing MAIN parents, join one component to itself, or infer an edge.
    fn try_restore_static_cut_parent(&mut self, child: usize,
        out: &mut StaticClearMaintenance) -> bool
    {
        let Some(parent) = self.static_cut_parents.get(&child).copied() else { return false; };
        out.historical_edges_examined += 1;
        if !self.static_validity_checker.is_state_valid(self.node(child).state())
            || !self.static_validity_checker.is_state_valid(self.node(parent).state()) {
            return false;
        }
        if self.node(parent).tree_id == Self::PRUNED_TREE_ID {
            // Parent restoration later wakes this child through the reverse
            // index. Do not requeue a waiting child forever on every tick.
            self.enqueue_static_clear_node(parent);
            return false;
        }
        if self.node(child).tree_id == self.node(parent).tree_id {
            self.forget_static_cut_parent(child);
            return false;
        }
        if !self.static_validity_checker.is_edge_valid(
            self.node(parent).state(), self.node(child).state()) { return false; }
        // Preserve an already reconnected MAIN child; reuse the same historical
        // edge in reverse to absorb its still-disconnected old-parent component.
        if self.node(child).tree_id == Self::MAIN_TREE_ID {
            self.reconnect_nodes(child, parent);
            self.optimization_start_nodes.clear();
            self.forget_static_cut_parent(child);
            self.wake_static_cut_children(child);
            self.wake_static_cut_children(parent);
            out.historical_edges_restored += 1;
            return true;
        }
        // Both components are live and distinct; child cannot contain MAIN.
        // Keep the actual historical direction, including between two forests.
        self.make_root_of_subtree(child);
        self.set_node_parent(child, Some(parent));
        let edge = self.node(parent).state().euclidean_distance(self.node(child).state());
        let cost = self.node(parent).cumulative_cost + edge;
        let tree = self.node(parent).tree_id;
        self.node_mut(child).edge_cost = edge;
        self.node_mut(child).cumulative_cost = cost;
        self.node_mut(child).tree_id = tree;
        self.propagate_info_to_children(child);
        self.forget_static_cut_parent(child);
        self.wake_static_cut_children(child);
        out.historical_edges_restored += 1;
        true
    }

    /// Queue bounded spatial neighborhoods affected by observed obstacle removal.
    /// `radius` covers the removed cells and robot footprint. Expand by the
    /// native reconnection neighborhood so both sides of a cut can participate.
    /// This is map maintenance, not a new sampling planner or an LRZ expansion.
    pub fn queue_static_clear_regions(&mut self, regions: &[(RealVectorState<F, N>, F)]) {
        for (center, radius) in regions {
            if !radius.is_finite() || *radius < F::zero()
                || center.values().iter().any(|v| !v.is_finite()) { continue; }
            let mut key: Vec<u64> = center.values().iter().map(|v| v.to_f64().unwrap().to_bits()).collect();
            key.push(radius.to_f64().unwrap().to_bits());
            if self.static_clear_region_keys.insert(key.clone()) {
                self.static_clear_regions.push_back((key.clone(),*center,*radius));
            }
            if (N == 2 || N == 3) && *radius > F::zero()
                && self.static_clear_sampling_regions.len() < Self::CLEAR_SAMPLING_MAX_REGIONS
                && self.static_clear_sampling_keys.insert(key.clone()) {
                self.static_clear_sampling_regions.push_back((key,*center,*radius,0,0,false));
            }
        }
    }

    // Use a fixed finite stencil for deterministic bounded coverage. Fixed
    // positions make repeated equal clear events idempotent under the spacing
    // check, including PRUNED samples that must be recovered instead of copied.
    pub fn maintain_static_clear_coverage(&mut self, max_attempts: usize,
        max_added_nodes: usize, max_total_nodes: usize, time_budget: Duration) -> StaticClearMaintenance
    {
        let mut out = StaticClearMaintenance::default();
        let started = Instant::now();
        let max_connections = 32;
        let max_attempts = max_attempts.min(Self::CLEAR_SAMPLING_MAX_ATTEMPTS);
        let max_added_nodes = max_added_nodes.min(Self::CLEAR_SAMPLING_MAX_ADDED);
        let max_total_nodes = max_total_nodes.min(Self::CLEAR_SAMPLING_MAX_ARENA);
        // Must be called after current static deltas and ordinary recovery.
        self.cpr_obstacles.clear(); self.robot_inside_ohzs.clear(); self.max_cpr_radius=F::zero();
        const OFFSETS: [(f64,f64);13] = [(0.,0.),(0.5,0.),(-0.5,0.),(0.,0.5),(0.,-0.5),
            (0.5,0.5),(-0.5,0.5),(0.5,-0.5),(-0.5,-0.5),(1.,0.),(-1.,0.),(0.,1.),(0.,-1.)];
        let mut offsets: Vec<[f64;N]> = Vec::new();
        if N == 2 {
            for (x,y) in OFFSETS { let mut a=[0.;N];a[0]=x;a[1]=y;offsets.push(a); }
        } else if N == 3 {
            offsets.push([0.;N]);
            for x in [-0.5,0.,0.5] { for y in [-0.5,0.,0.5] { for z in [-0.5,0.,0.5] {
                if x==0. && y==0. && z==0. {continue;}
                let mut a=[0.;N];a[0]=x;a[1]=y;a[2]=z;offsets.push(a);
            }}}
            for axis in 0..N {for sign in [-1.,1.] {let mut a=[0.;N];a[axis]=sign;offsets.push(a);}}
        }
        let spacing = self.hot_node_neighborhood_radius * F::from(0.5).unwrap() + F::from(1e-4).unwrap();
        while (N == 2 || N == 3) && out.sampling_attempts < max_attempts
            && out.sampling_added_nodes < max_added_nodes
            && self.nodes.len() < max_total_nodes
            && out.connections < max_connections && started.elapsed() < time_budget {
            let Some((key,center,radius,index,pass,mut progressed)) = self.static_clear_sampling_regions.pop_front() else {break;};
            out.sampling_attempts += 1;
            // Test every fixed position independently: an occupied center must
            // not veto reachable free parts at the same cleared tile's edge.
            let mut values = *center.values();
            for axis in 0..N {values[axis] = values[axis] + radius * F::from(offsets[index][axis]).unwrap();}
            let candidate = RealVectorState::new(values);
            let added = (|| {
            if !self.static_validity_checker.is_state_valid(&candidate) {return false;}
            // Existing samples, including presently pruned ones, own their space.
            // Never duplicate a sample while waiting for the old-parent queue.
            if !self.nearest_neighbors.within_radius(&candidate,spacing).is_empty() {return false;}
            let probe = Node::new(candidate,None,F::infinity(),F::infinity(),i32::MAX);
            let neighbors = self.get_feasible_neighbors_of_different_tree(&probe);
            if neighbors.is_empty() || neighbors.len()>32 {return false;}
            // If old samples can bridge directly, wake that ordinary work instead
            // of adding a point merely because its recovery turn has not arrived.
            let mut direct_bridge = false;
            for (i,&a) in neighbors.iter().enumerate() {
                for &b in &neighbors[i+1..] {
                    if self.node(a).tree_id != self.node(b).tree_id
                        && self.node(a).state().euclidean_distance(self.node(b).state())
                            <= self.hot_node_neighborhood_radius
                        && self.static_validity_checker.is_edge_valid(self.node(a).state(),self.node(b).state()) {
                        self.enqueue_static_clear_node(a);self.enqueue_static_clear_node(b);
                        direct_bridge = true;
                    }
                }
            }
            if direct_bridge {return false;}
            let components: BTreeSet<_> = neighbors.iter().map(|&id|self.node(id).tree_id).collect();
            if components.len() > max_connections-out.connections {return false;}
            // Native insertion only joins different components; MAIN is always
            // the parent. Clear its optimization work without rewiring MAIN,
            // rebuilding current_path, or issuing any motion certificate.
            if self.add_new_node_at_state_and_connect(candidate) {
                self.optimization_start_nodes.clear();
                out.sampling_added_nodes += 1;
                out.connections += components.len();
                for id in neighbors {
                    self.wake_static_cut_children(id);
                    self.enqueue_static_clear_node(id);
                }
                return true;
            }
            false
            })();
            progressed |= added;
            if index+1 < offsets.len() {
                self.static_clear_sampling_regions.push_back((key,center,radius,index+1,pass,progressed));
            } else if progressed && pass+1 < offsets.len() {
                // A new frontier point may make an earlier unsupported center
                // reachable. Revisit this same finite stencil only after real
                // local growth; no-progress and the finite stencil-pass ceiling both stop it.
                self.static_clear_sampling_regions.push_back((key,center,radius,0,pass+1,false));
            } else {self.static_clear_sampling_keys.remove(&key);}
        }
        out.sampling_pending_regions = self.static_clear_sampling_regions.len();
        debug_assert!(self.optimization_start_nodes.is_empty());
        debug_assert!(self.debug_check_finalized_forest());
        out
    }

    /// Recover old samples and join existing components near explicit clear
    /// events, including while the robot itself is held in collision. The caller
    /// must first install the full current checker and apply all added obstacles.
    /// Every restored state and new edge uses that checker. Old parents are never
    /// restored blindly. MAIN parents are never rewired, so a retained path and
    /// execution cursor remain unchanged. No path or motion approval is produced.
    /// Limits bound work units; time is a soft bound around native primitives
    /// (one subtree cost/ID propagation cannot be interrupted midway).
    pub fn maintain_static_clear_regions(&mut self, max_regions: usize, max_nodes: usize,
        max_connections: usize, time_budget: Duration) -> StaticClearMaintenance
    {
        let mut out = StaticClearMaintenance::default();
        out.retained_cut_edges = self.static_cut_parents.len();
        out.historical_max_edge_length_m = self.static_cut_max_length().to_f64().unwrap();
        if self.static_clear_regions.is_empty() && self.static_clear_nodes.is_empty() { return out; }
        debug_assert!(self.pruned_nodes.is_empty() && self.disjoint_tree_roots.is_empty());
        debug_assert!(self.optimization_start_nodes.is_empty());
        let started = Instant::now();
        // Last cycle's CPR is not the current global static domain. The next
        // dynamic update rebuilds CPR before any executable-path validation.
        self.cpr_obstacles.clear(); self.robot_inside_ohzs.clear(); self.max_cpr_radius = F::zero();
        let region_budget = time_budget / 3;
        while out.regions_examined < max_regions && started.elapsed() < region_budget {
            let Some((key,center,radius)) = self.static_clear_regions.pop_front() else { break; };
            self.static_clear_region_keys.remove(&key);
            let query_started = Instant::now();
            let mut ids=self.nearest_neighbors.within_radius(&center,radius+self.hot_node_neighborhood_radius.max(self.static_cut_max_length()));
            ids.sort_unstable();
            for id in ids {
                if self.static_clear_node_ids.insert(id) {self.static_clear_nodes.push_back(id);}
            }
            out.region_query_time_us = out.region_query_time_us.saturating_add(
                query_started.elapsed().as_micros().min(u64::MAX as u128) as u64);
            out.regions_examined += 1;
        }
        // A clear-region query contains many already-connected MAIN samples.
        // Stable priority keeps old, detached samples from waiting behind them,
        // but MAIN entries remain available: a MAIN point near the clear region
        // may be the only queued endpoint of a cross-boundary reconnection.
        let mut recovery = VecDeque::new();
        let mut main = VecDeque::new();
        while let Some(id) = self.static_clear_nodes.pop_front() {
            if self.node(id).tree_id == Self::MAIN_TREE_ID { main.push_back(id); }
            else { recovery.push_back(id); }
        }
        let mut recovery_run = self.static_clear_priority_streak;
        while out.nodes_examined < max_nodes && out.connections < max_connections
            && started.elapsed() < time_budget {
            // Other connections may have absorbed a queued component already.
            // Keep its MAIN opportunity, without occupying recovery priority.
            while recovery.front().map_or(false, |id| self.node(*id).tree_id == Self::MAIN_TREE_ID) {
                main.push_back(recovery.pop_front().unwrap());
            }
            let id = if !recovery.is_empty() && (recovery_run < 8 || main.is_empty()) {
                recovery_run += 1;
                recovery.pop_front()
            } else {
                recovery_run = 0;
                main.pop_front()
            };
            let Some(id) = id else { break; };
            self.static_clear_node_ids.remove(&id); out.nodes_examined += 1;
            if !self.static_validity_checker.is_state_valid(self.node(id).state()) { continue; }
            if self.node(id).tree_id == Self::PRUNED_TREE_ID {
                if self.node(id).parent.is_some() || !self.node(id).children.is_empty() { continue; }
                if id == 0 { // normally already recovered by the checker setter
                    self.node_mut(id).tree_id = Self::MAIN_TREE_ID;
                    self.node_mut(id).edge_cost = F::zero();
                    self.node_mut(id).cumulative_cost = F::zero();
                } else {
                    self.tree_id_counter += 1;
                    self.node_mut(id).tree_id = self.tree_id_counter;
                    self.node_mut(id).edge_cost = F::infinity();
                    self.node_mut(id).cumulative_cost = F::infinity();
                }
                out.restored_nodes += 1;
                self.wake_static_cut_children(id);
            }
            if self.try_restore_static_cut_parent(id, &mut out) {
                out.connections += 1;
                self.enqueue_static_clear_node(id);
                continue;
            }
            let neighbors = self.get_feasible_neighbors_of_different_tree(self.node(id));
            // Prefer an available MAIN connection, then the nearest native
            // neighbor. Each connection reduces the number of components.
            let neighbor = neighbors.into_iter().min_by_key(|&other| (
                self.node(other).tree_id != Self::MAIN_TREE_ID,
                OrderedFloat::from(self.node(id).state().euclidean_distance_squared(self.node(other).state())), other));
            if let Some(other) = neighbor {
                self.reconnect_nodes(id, other);
                // Reconnection itself normalizes costs. Do not run optimization:
                // it could rewire MAIN/path edges during read-side maintenance.
                self.optimization_start_nodes.clear();
                self.wake_static_cut_children(id);
                self.wake_static_cut_children(other);
                out.connections += 1;
                if self.static_clear_node_ids.insert(id) {self.static_clear_nodes.push_back(id);} // another component may remain
            }
        }
        self.static_clear_priority_streak = recovery_run;
        // Native restoration may enqueue dependent parent/child work during
        // this pass. Keep it reachable on the next call without dropping IDs.
        while let Some(id) = self.static_clear_nodes.pop_front() {
            if self.node(id).tree_id == Self::MAIN_TREE_ID { main.push_back(id); }
            else { recovery.push_back(id); }
        }
        recovery.append(&mut main);
        self.static_clear_nodes = recovery;
        out.retained_cut_edges = self.static_cut_parents.len();
        out.historical_max_edge_length_m = self.static_cut_max_length().to_f64().unwrap();
        out.pending_regions = self.static_clear_regions.len();
        out.pending_nodes = self.static_clear_nodes.len();
        debug_assert!(self.debug_check_finalized_forest());
        out
    }

    /// Grow the existing SMART tree by running additional RRT* sampling iterations.
    ///
    /// Used as a fast alternative to full `build_smart()` when an agent is stuck.
    /// The existing tree structure is preserved; new nodes are added and wired into
    /// the main tree using the same extend/rewire logic as the initial RRT* build.
    ///
    /// After sampling, attempts to build a path via `build_planned_path()`.
    ///
    /// Parameters:
    /// - `additional_iters`: Number of new RRT* sample attempts.
    /// - `current_robot_state`: The robot's current position (needed by `build_planned_path`).
    ///
    /// Returns `true` if a valid path was found after growing.
    pub fn grow_tree(
        &mut self,
        additional_iters: usize,
        current_robot_state: RealVectorState<F, N>,
    ) -> bool {
        assert!(
            !self.initial_planning_phase,
            "grow_tree only valid after initial planning"
        );

        // Update current state so build_planned_path searches from the right place
        self.current_state = current_robot_state;

        // Clear CPR state (no dynamic obstacles during stuck rebuild)
        self.cpr_obstacles.clear();
        self.max_cpr_radius = F::zero();
        self.robot_inside_ohzs.clear();

        let steering_range = self.lsr_initial_radius; // matches initial RRT* steering_range (1.0)
        let rewire_radius = self.hot_node_neighborhood_radius;

        for _ in 0..additional_iters {
            // 1. Sample
            let sample = self.sampling_distribution.sample();
            if !self.static_validity_checker.is_state_valid(&sample) {
                continue;
            }

            // 2. Find nearest existing node
            let nearest_id = match self.nearest_neighbors.nearest_one(&sample) {
                Some(id) => id,
                None => continue,
            };
            if self.nodes[nearest_id].tree_id == Self::PRUNED_TREE_ID {
                continue;
            }

            // 3. Steer towards sample (cap at steering_range)
            let nearest_state = self.nodes[nearest_id].state.clone();
            let direction = &sample - &nearest_state;
            let distance = direction.norm();
            let (new_state, steer_dist) = if distance > steering_range {
                (
                    &nearest_state + &(&direction / distance * steering_range),
                    steering_range,
                )
            } else {
                (sample, distance)
            };

            if steer_dist < F::from(0.001).unwrap() {
                continue;
            }

            // 4. Validity checks
            if !self.static_validity_checker.is_state_valid(&new_state) {
                continue;
            }
            if !self
                .static_validity_checker
                .is_edge_valid(&nearest_state, &new_state)
            {
                continue;
            }

            // 5. Find best parent among neighbors (RRT* rewire-to-best-parent)
            let neighbors = self
                .nearest_neighbors
                .within_radius(&new_state, rewire_radius);

            let mut best_parent_id = nearest_id;
            let mut best_cost = self.nodes[nearest_id].cumulative_cost + steer_dist;
            let mut best_edge_cost = steer_dist;

            // Only connect to main-tree nodes with finite cost
            if self.nodes[nearest_id].tree_id != Self::MAIN_TREE_ID
                || self.nodes[nearest_id].cumulative_cost == F::infinity()
            {
                // Nearest is not usable; search neighbors for a main-tree parent
                best_cost = F::infinity();
            }

            for &nb_id in &neighbors {
                if self.nodes[nb_id].tree_id != Self::MAIN_TREE_ID {
                    continue;
                }
                if self.nodes[nb_id].cumulative_cost == F::infinity() {
                    continue;
                }
                let nb_edge = new_state.euclidean_distance(&self.nodes[nb_id].state);
                let nb_cost = self.nodes[nb_id].cumulative_cost + nb_edge;
                if nb_cost < best_cost
                    && self
                        .static_validity_checker
                        .is_edge_valid(&self.nodes[nb_id].state, &new_state)
                {
                    best_parent_id = nb_id;
                    best_cost = nb_cost;
                    best_edge_cost = nb_edge;
                }
            }

            if best_cost == F::infinity() {
                continue; // No valid main-tree parent found
            }

            // 6. Add node (parentless first, then wire via set_node_parent)
            let new_node = Node::new(
                new_state.clone(),
                None, // parent set below
                best_edge_cost,
                best_cost,
                Self::MAIN_TREE_ID,
            );
            let new_node_id = self.add_node(new_node);
            self.set_node_parent(new_node_id, Some(best_parent_id));

            // 7. Rewire neighbors (RRT* rewire-neighbors)
            for &nb_id in &neighbors {
                if nb_id == best_parent_id || nb_id == new_node_id {
                    continue;
                }
                if self.nodes[nb_id].tree_id != Self::MAIN_TREE_ID {
                    continue;
                }
                let nb_edge = new_state.euclidean_distance(&self.nodes[nb_id].state);
                let new_nb_cost = best_cost + nb_edge;
                if new_nb_cost < self.nodes[nb_id].cumulative_cost
                    && self
                        .static_validity_checker
                        .is_edge_valid(&new_state, &self.nodes[nb_id].state)
                {
                    self.set_node_parent(nb_id, Some(new_node_id));
                    self.nodes[nb_id].edge_cost = nb_edge;
                    self.nodes[nb_id].cumulative_cost = new_nb_cost;
                    self.propagate_info_to_children(nb_id);
                }
            }
        }

        // Try to build a path with the enriched tree
        self.current_path = self.build_planned_path();
        self.current_path.is_some()
    }

    /// Senior baseline: path-only repair.
    ///
    /// Unlike `update_static_obstacles()` which scans ALL nodes/edges in the tree,
    /// this only checks nodes and edges ON the current robot→goal path.
    /// Off-path dirty edges are left untouched.
    ///
    /// If the path is broken and hot-node reconnect cannot fix it within the
    /// timeout budget, returns `ReplanningFailed` (caller should do full rebuild).
    ///
    /// IMPORTANT: Does NOT use `sampling_tree_repair_iteration()` because sampling
    /// into a dirty tree causes unbounded node growth and memory explosion.
    pub fn update_static_obstacles_path_only<TC>(
        &mut self,
        new_static_validity_checker: Box<dyn ValidityChecker<F, N>>,
        mut termination_condition: Option<&mut TC>,
        current_robot_state: RealVectorState<F, N>,
    ) -> SmartUpdateResult
    where
        TC: TerminationCondition,
    {
        assert!(
            !self.initial_planning_phase,
            "Function only valid for non-initial planning"
        );

        self.current_state = current_robot_state;

        // Get current path node IDs BEFORE replacing checker
        let path_node_ids: Vec<usize> = match &self.current_path {
            Some(p) => p.clone(),
            None => {
                // No path exists, just swap checker and fail
                self.static_validity_checker = new_static_validity_checker;
                return SmartUpdateResult::ReplanningFailed;
            }
        };

        // Replace checker
        let _old = std::mem::replace(
            &mut self.static_validity_checker,
            new_static_validity_checker,
        );

        // Clear CPR state
        self.cpr_obstacles.clear();
        self.max_cpr_radius = F::zero();
        self.robot_inside_ohzs.clear();

        // Reset replanning state
        // Retained static forest components keep their IDs across repair calls.
        self.replanning_triggered_by = Some(ReplanningTriggerCondition::PathUnsafe);
        assert!(self.disjoint_tree_roots.is_empty());
        assert!(self.pruned_nodes.is_empty());
        assert!(self.optimization_start_nodes.is_empty());

        // Phase 1: Prune PATH nodes inside new walls
        let mut pruned_nodes = Vec::new();
        for &node_id in &path_node_ids {
            if self.node(node_id).tree_id == Self::PRUNED_TREE_ID {
                continue;
            }
            if !self
                .static_validity_checker
                .is_state_valid(self.node(node_id).state())
            {
                self.node_mut(node_id).tree_id = Self::PRUNED_TREE_ID;
                let old_parent_id = self.node(node_id).parent;
                self.set_node_parent(node_id, None);
                self.node_mut(node_id).cumulative_cost = F::infinity();
                pruned_nodes.push((node_id, old_parent_id));
            }
        }

        // Phase 2: Create disjoint subtrees from children of pruned path nodes
        let mut disjoint_tree_roots = Vec::new();
        for &(node_id, _) in &pruned_nodes {
            let children = self.node(node_id).children.clone();
            for child_id in children {
                if self.node(child_id).tree_id == Self::PRUNED_TREE_ID {
                    continue;
                }
                self.tree_id_counter += 1;
                let new_tree_id = self.tree_id_counter;
                let old_parent_id = self.node(child_id).parent;
                self.set_node_parent(child_id, None);
                let child = self.node_mut(child_id);
                child.cumulative_cost = F::infinity();
                child.tree_id = new_tree_id;
                self.propagate_info_to_children(child_id);
                disjoint_tree_roots.push((child_id, old_parent_id));
            }
        }

        // Phase 3: Break PATH edges that cross new walls (only consecutive path pairs)
        for pair in path_node_ids.windows(2) {
            let child_id = pair[0];
            let parent_id = pair[1];
            if self.node(child_id).tree_id == Self::PRUNED_TREE_ID {
                continue;
            }
            if self.node(parent_id).tree_id == Self::PRUNED_TREE_ID {
                continue;
            }
            // Check if this edge is still a parent-child relationship
            if self.node(child_id).parent != Some(parent_id) {
                continue;
            }
            if !self
                .static_validity_checker
                .is_edge_valid(self.node(parent_id).state(), self.node(child_id).state())
            {
                let old_parent_id = Some(parent_id);
                self.set_node_parent(child_id, None);
                self.tree_id_counter += 1;
                self.node_mut(child_id).tree_id = self.tree_id_counter;
                self.node_mut(child_id).cumulative_cost = F::infinity();
                self.propagate_info_to_children(child_id);
                disjoint_tree_roots.push((child_id, old_parent_id));
            }
        }

        self.pruned_nodes = pruned_nodes;
        self.disjoint_tree_roots = disjoint_tree_roots;

        let num_pruned = self.pruned_nodes.len();
        let num_disjoint = self.disjoint_tree_roots.len();

        if num_pruned == 0 && num_disjoint == 0 {
            return SmartUpdateResult::NoReplanningNeeded;
        }

        // Phase 4: Hot-node repair (NO sampling fallback)
        let hotnode_search_center = self.current_state.clone();
        self.current_path = self.build_planned_path();

        while self.current_path.is_none() {
            // Timeout check
            if let Some(tc) = termination_condition.as_mut() {
                if tc.evaluate() {
                    self.tree_optimization();
                    self.current_path = self.build_planned_path();
                    self.form_single_tree_after_static_update();
                    return SmartUpdateResult::ReplanningFailed;
                }
            }

            let best_hot_node = self.find_hot_node_with_expanding_lsr(&hotnode_search_center);

            if best_hot_node.is_some() {
                self.reconnect_best_hot_node(&best_hot_node.unwrap());
            } else {
                // Senior baseline: NO sampling. Can't fix → fail, let caller rebuild.
                self.tree_optimization();
                self.current_path = self.build_planned_path();
                self.form_single_tree_after_static_update();
                return SmartUpdateResult::ReplanningFailed;
            }

            self.current_path = self.build_planned_path();
        }

        // Phase 5: Optimize and finalize
        self.tree_optimization();
        self.current_path = self.build_planned_path();
        self.form_single_tree_after_static_update();

        SmartUpdateResult::ReplanningSuccessful
    }

    /// Lightweight edge cleanup: check all parent edges against the given checker
    /// and break any that are now invalid. Creates disjoint subtrees but does NOT
    /// do hot-node repair or path rebuilding. Used in Hybrid mode when new walls
    /// don't block the current path but may invalidate some tree edges.
    pub fn cleanup_edges_against_checker(&mut self, checker: &dyn ValidityChecker<F, N>) {
        for node_id in 0..self.nodes.len() {
            if self.node(node_id).tree_id == Self::PRUNED_TREE_ID {
                continue;
            }
            if let Some(parent_id) = self.node(node_id).parent {
                if self.node(parent_id).tree_id == Self::PRUNED_TREE_ID {
                    continue;
                }
                if !checker.is_edge_valid(self.node(parent_id).state(), self.node(node_id).state())
                {
                    // Break the edge — child becomes a disconnected subtree root
                    self.set_node_parent(node_id, None);
                    // ★ FIX: mark subtree as non-MAIN so descendants can't become new parent (cycle prevention)
                    self.tree_id_counter += 1;
                    let temp_id = self.tree_id_counter;
                    self.node_mut(node_id).tree_id = temp_id;
                    self.node_mut(node_id).cumulative_cost = F::infinity();
                    self.propagate_info_to_children(node_id);
                    // Try to reconnect via KD-tree neighbors in main tree
                    let neighbors = self.nearest_neighbors.within_radius(
                        self.node(node_id).state(),
                        self.hot_node_neighborhood_radius,
                    );
                    let mut best: Option<(usize, F)> = None;
                    for &nb_id in &neighbors {
                        if nb_id == node_id {
                            continue;
                        }
                        if self.node(nb_id).tree_id != Self::MAIN_TREE_ID {
                            continue;
                        }
                        if !self
                            .static_validity_checker
                            .is_edge_valid(self.node(nb_id).state(), self.node(node_id).state())
                        {
                            continue;
                        }
                        let cost = self.node(nb_id).cumulative_cost
                            + self
                                .node(nb_id)
                                .state()
                                .euclidean_distance(self.node(node_id).state());
                        if best.is_none() || cost < best.unwrap().1 {
                            best = Some((nb_id, cost));
                        }
                    }
                    if let Some((nb_id, _)) = best {
                        self.set_node_parent(node_id, Some(nb_id));
                        let edge_cost = self
                            .node(nb_id)
                            .state()
                            .euclidean_distance(self.node(node_id).state());
                        self.node_mut(node_id).edge_cost = edge_cost;
                        self.node_mut(node_id).cumulative_cost =
                            self.node(nb_id).cumulative_cost + edge_cost;
                        self.node_mut(node_id).tree_id = Self::MAIN_TREE_ID;
                        self.propagate_info_to_children(node_id);
                    }
                    // If no neighbor found, node stays parentless — won't be used for paths
                    // but won't cause issues either.
                }
            }
        }
    }

    // Check the remaining ordered path, including the measured connector. This
    // is not an arena scan: full-tree pruning runs only when static evidence is
    // applied. Known static obstacles constrain the entire candidate path while
    // dynamic CPR collision checks retain their local reaction region.
    fn is_remaining_path_static_valid(&self, next: usize) -> bool {
        let path = match self.current_path.as_ref() {
            Some(path) if next < path.len() => path,
            _ => return false,
        };
        let mut start = &self.current_state;
        for &id in &path[next..] {
            let node = self.node(id);
            let end = node.state();
            if node.tree_id != Self::MAIN_TREE_ID
                || !self.static_validity_checker.is_state_valid(end)
                || !self.static_validity_checker.is_edge_valid(start, end)
            { return false; }
            start = end;
        }
        true
    }

    // Static finalization can reconnect a valid orphan even after the sampling
    // budget expires. Publish the path/status from that finalized forest.
    fn finish_static_repair(&mut self) -> SmartUpdateResult {
        self.form_single_tree_after_static_update();
        self.current_path = self.build_planned_path();
        if self.current_path.is_some() {
            SmartUpdateResult::ReplanningSuccessful
        } else {
            SmartUpdateResult::ReplanningFailed
        }
    }

    /// Form a single tree after static obstacle update.
    /// Unlike form_single_tree() for dynamic obstacles:
    /// - Currently invalid samples remain pruned until explicit clear maintenance
    /// - Disjoint roots are only reconnected if edge is valid with new static checker
    /// - Remaining orphans are absorbed by walking parent chains
    fn form_single_tree_after_static_update(&mut self) {
        // Retain only candidate references; active invalid edges remain cut.
        let cuts: Vec<_> = self.pruned_nodes.iter().chain(self.disjoint_tree_roots.iter())
            .copied().collect();
        for (child, parent) in cuts {
            if let Some(parent) = parent { self.remember_static_cut_parent(child, parent); }
        }
        self.pruned_nodes.clear();

        // Try to reconnect disjoint tree roots
        let disjoint_tree_roots = std::mem::take(&mut self.disjoint_tree_roots);
        for &(node_id, old_parent_id) in &disjoint_tree_roots {
            if self.node(node_id).tree_id == Self::MAIN_TREE_ID {
                continue; // Already reconnected during repair
            }
            if self.node(node_id).tree_id == Self::PRUNED_TREE_ID {
                continue; // Safety: skip if somehow pruned
            }

            // Try old parent if edge is valid with new walls
            if let Some(old_pid) = old_parent_id {
                if self.node(old_pid).tree_id == Self::MAIN_TREE_ID
                    && self
                        .static_validity_checker
                        .is_edge_valid(self.node(old_pid).state(), self.node(node_id).state())
                {
                    self.make_root_of_subtree(node_id);
                    self.set_node_parent(node_id, Some(old_pid));

                    let edge_cost = self
                        .node(old_pid)
                        .state()
                        .euclidean_distance(self.node(node_id).state());
                    self.node_mut(node_id).edge_cost = edge_cost;
                    self.node_mut(node_id).cumulative_cost =
                        self.node(old_pid).cumulative_cost + edge_cost;
                    self.node_mut(node_id).tree_id = Self::MAIN_TREE_ID;
                    self.propagate_info_to_children(node_id);
                    continue;
                }
            }

            // Try finding any main-tree neighbor with a valid edge
            // Use a wider search radius (2x hot_node_neighborhood) for better reconnection
            let search_radius = self.hot_node_neighborhood_radius * F::from(2.0).unwrap();
            let neighbors = self
                .nearest_neighbors
                .within_radius(self.node(node_id).state(), search_radius);
            let mut best_neighbor: Option<(usize, F)> = None;
            for &nb_id in &neighbors {
                if self.node(nb_id).tree_id != Self::MAIN_TREE_ID {
                    continue;
                }
                if self
                    .static_validity_checker
                    .is_edge_valid(self.node(nb_id).state(), self.node(node_id).state())
                {
                    let cost = self.node(nb_id).cumulative_cost
                        + self
                            .node(nb_id)
                            .state()
                            .euclidean_distance(self.node(node_id).state());
                    if best_neighbor.is_none() || cost < best_neighbor.unwrap().1 {
                        best_neighbor = Some((nb_id, cost));
                    }
                }
            }
            if let Some((nb_id, _)) = best_neighbor {
                self.make_root_of_subtree(node_id);
                self.set_node_parent(node_id, Some(nb_id));
                let edge_cost = self
                    .node(nb_id)
                    .state()
                    .euclidean_distance(self.node(node_id).state());
                self.node_mut(node_id).edge_cost = edge_cost;
                self.node_mut(node_id).cumulative_cost =
                    self.node(nb_id).cumulative_cost + edge_cost;
                self.node_mut(node_id).tree_id = Self::MAIN_TREE_ID;
                self.propagate_info_to_children(node_id);
            }
            // If no neighbor found, subtree remains disconnected (non-main tree_id).
            // These nodes won't be used for path planning but also won't cause issues.
        }

        // Iterative absorption: walk through all non-main, non-pruned nodes.
        // If a node's existing parent is in the main tree and edge is valid, absorb it.
        // This catches nodes in subtrees that were partially reconnected above.
        let mut changed = true;
        while changed {
            changed = false;
            for node_id in 0..self.nodes.len() {
                let tree_id = self.node(node_id).tree_id;
                if tree_id == Self::MAIN_TREE_ID || tree_id == Self::PRUNED_TREE_ID {
                    continue;
                }
                if let Some(parent_id) = self.node(node_id).parent {
                    if self.node(parent_id).tree_id == Self::MAIN_TREE_ID
                        && self
                            .static_validity_checker
                            .is_edge_valid(self.node(parent_id).state(), self.node(node_id).state())
                    {
                        let edge_cost = self
                            .node(parent_id)
                            .state()
                            .euclidean_distance(self.node(node_id).state());
                        self.node_mut(node_id).tree_id = Self::MAIN_TREE_ID;
                        self.node_mut(node_id).edge_cost = edge_cost;
                        self.node_mut(node_id).cumulative_cost =
                            self.node(parent_id).cumulative_cost + edge_cost;
                        self.propagate_info_to_children(node_id);
                        changed = true;
                    }
                }
            }
        }
    }

    /// Rebuild only the actual-current connector and path on the retained forest.
    pub fn refresh_current_path(&mut self, current: RealVectorState<F,N>) {
        self.current_state=current;
        self.current_path=self.build_planned_path();
    }

    /// Change the goal without running a new RRT*. Existing node coordinates
    /// and undirected valid edges survive. ID 0 is reserved for the goal by
    /// SMART; its previous node moves to returned ID. This O(arena+edges) pass
    /// reverses parent orientation, rebuilds costs/NN, and preserves cut history.
    pub fn reroot_goal(&mut self, goal: RealVectorState<F,N>, current: RealVectorState<F,N>)
        -> Result<(usize,usize,usize,bool), &'static str> {
        if !self.static_validity_checker.is_state_valid(&goal) {return Err("goal_blocked");}
        let count=self.nodes.len();
        if count==0 || count>=Self::CLEAR_SAMPLING_MAX_ARENA {return Err("tree_capacity");}
        let remap=|id:usize| if id==0 {count} else {id};
        let cuts:Vec<_>=self.static_cut_parents.iter().map(|(&c,&p)|(remap(c),remap(p))).collect();
        let old=self.nodes.clone();
        self.nodes.push(old[0].clone());
        self.nodes[0]=Node::new(goal,None,F::zero(),F::zero(),Self::MAIN_TREE_ID);
        let mut valid=vec![true;count+1];
        for id in 1..=count { valid[id]=self.nodes[id].tree_id!=Self::PRUNED_TREE_ID
            && self.static_validity_checker.is_state_valid(self.nodes[id].state()); }
        let mut graph:Vec<Vec<usize>>=vec![Vec::new();count+1];
        let mut retained_edges=0;
        for (id,node) in old.iter().enumerate() {if let Some(p)=node.parent {
            let (a,b)=(remap(id),remap(p));
            if valid[a] && valid[b] && self.static_validity_checker.is_edge_valid(self.nodes[a].state(),self.nodes[b].state()) {
                graph[a].push(b);graph[b].push(a);retained_edges+=1;
            }
        }}
        let mut candidates:Vec<_>=(1..=count).filter(|&i|valid[i]).collect();
        candidates.sort_by(|&a,&b| self.nodes[a].state().euclidean_distance(&goal)
            .partial_cmp(&self.nodes[b].state().euclidean_distance(&goal)).unwrap());
        let connected=candidates.into_iter().find(|&i|self.static_validity_checker.is_edge_valid(&goal,self.nodes[i].state()));
        if let Some(i)=connected {graph[0].push(i);graph[i].push(0);}
        for node in &mut self.nodes {node.parent=None;node.children.clear();node.tree_id=Self::PRUNED_TREE_ID;
            node.edge_cost=F::infinity();node.cumulative_cost=F::infinity();}
        let mut seen=vec![false;count+1];
        for root in 0..=count {if seen[root] || !valid[root] {continue;}
            let tree_id=if root==0 {Self::MAIN_TREE_ID} else {self.tree_id_counter+=1;self.tree_id_counter};
            self.nodes[root].tree_id=tree_id;self.nodes[root].edge_cost=F::zero();
            self.nodes[root].cumulative_cost=if root==0 {F::zero()} else {F::infinity()};
            seen[root]=true;let mut queue=VecDeque::from([root]);
            while let Some(parent)=queue.pop_front() {for &child in &graph[parent] {
                if seen[child] {continue;}seen[child]=true;
                let edge=self.nodes[parent].state().euclidean_distance(self.nodes[child].state());
                self.nodes[child].parent=Some(parent);self.nodes[parent].children.push(child);
                self.nodes[child].tree_id=tree_id;self.nodes[child].edge_cost=edge;
                self.nodes[child].cumulative_cost=self.nodes[parent].cumulative_cost+edge;queue.push_back(child);
            }}
        }
        self.static_cut_parents.clear();self.static_cut_children.clear();self.static_cut_edge_lengths.clear();
        for(c,p)in cuts {self.remember_static_cut_parent(c,p);}
        // Spatial region queues stay valid. Remap queued node IDs explicitly.
        self.static_clear_nodes=self.static_clear_nodes.iter().map(|&i|remap(i)).collect();
        self.static_clear_node_ids=self.static_clear_node_ids.iter().map(|&i|remap(i)).collect();
        self.cpr_obstacles.clear();self.robot_inside_ohzs.clear();self.max_cpr_radius=F::zero();
        self.pruned_nodes.clear();self.disjoint_tree_roots.clear();self.optimization_start_nodes.clear();
        self.nearest_neighbors=NN::new();
        for(id,node)in self.nodes.iter().enumerate(){self.nearest_neighbors.add(node.state,id);}
        self.goal=goal;self.current_state=current;self.current_path=self.build_planned_path();
        debug_assert!(self.debug_check_finalized_forest());
        Ok((count,count,retained_edges,connected.is_some()))
    }

    pub fn get_path(&self) -> Option<Vec<RealVectorState<F, N>>> {
        let path_nodes = self.current_path.clone();
        if path_nodes.is_none() {
            return None;
        }
        let path_nodes = path_nodes.unwrap();
        let mut path = Vec::new();
        for &node_id in path_nodes.iter() {
            path.push(self.node(node_id).state().clone());
        }
        Some(path)
    }

    /// Shortcut the current internal path by randomly trying to replace segments
    /// with straight lines. Operates directly on `self.current_path` (the node ID
    /// list), so `get_path()` automatically returns the shortened version and
    /// `path_idx` stays consistent between the caller and SMART internals.
    ///
    /// Only checks the static validity checker (walls). Dynamic obstacles are
    /// re-checked every frame by `update()`, so skipping them here is safe.
    pub fn shortcut_current_path(&mut self, max_iterations: usize) {
        let path = match self.current_path.as_mut() {
            Some(p) if p.len() >= 3 => p,
            _ => return,
        };

        let mut rng = rand::thread_rng();
        for _ in 0..max_iterations {
            let len = path.len();
            if len < 3 {
                break;
            }
            let i = rng.gen_range(0..len - 2);
            let j = rng.gen_range(i + 2..len);
            let state_i = self.nodes[path[i]].state().clone();
            let state_j = self.nodes[path[j]].state().clone();
            if self
                .static_validity_checker
                .is_edge_valid(&state_i, &state_j)
            {
                path.drain(i + 1..j);
            }
        }
    }

    pub fn replanning_triggered_by(&self) -> Option<ReplanningTriggerCondition> {
        self.replanning_triggered_by.clone()
    }

    pub fn get_nodes(&self) -> &Vec<Node<F, N>> {
        &self.nodes
    }

    // Private functions
    // ----------------

    /// Builds the Critical Pruning Zone (CPR). Collisions with dynamic obstacles are checked only in this region.
    /// This function sets the `cpr_obstacles` and `max_cpr_radius` fields.
    /// `cpr_obstacles` contains the obstacles whose OHZ's interesect the LRZ.
    /// `max_cpr_radius` is the maximum distance between the current state and the furthest point of any CPR obstacle.
    fn build_cpr(&mut self, dynamic_obstacles: &Vec<SphericalObstacleWithOhz<F, N>>) {
        self.cpr_obstacles = Vec::new();
        self.robot_inside_ohzs = Vec::new();
        self.max_cpr_radius = self.lrz_radius; // At the very least, the nodes within LRZ must be checked for collision

        for obs in dynamic_obstacles {
            let hazard = obs.hazard();
            // The enclosing sphere is only a conservative LRZ/NN broadphase;
            // all subsequent collision and pruning predicates use exact OHZ.
            let dist = hazard.center().euclidean_distance(&self.current_state);
            if dist <= self.lrz_radius + hazard.radius() {
                if hazard.contains(&self.current_state) {
                    self.cpr_obstacles.push(hazard.for_robot_inside(
                        &self.current_state, self.ohz_shrinkage_margin));
                    self.robot_inside_ohzs.push(hazard);
                } else {
                    self.cpr_obstacles.push(hazard);
                }

                // The radius from the current state that has to be checked for collision with this obstacle
                let obstacle_coll_check_radius = self.cpr_obstacles.last().unwrap().radius() + dist;
                // Check if the radius is greater than the current max_cpr_radius
                if obstacle_coll_check_radius > self.max_cpr_radius {
                    self.max_cpr_radius = obstacle_coll_check_radius;
                }
            }
        }
    }

    /// Checks the ordered remaining path against CPR obstacles inside the LRZ.
    /// The cursor names the next path node, so the first segment begins at the
    /// measured current state. Tree parent pointers are not execution cursors.
    /// Returns the destination node of the first unsafe segment, when available.
    fn is_path_safe_within_lrz(&self, current_path_index: usize) -> (bool, Option<usize>) {
        let path = match self.current_path.as_ref() {
            Some(path) if current_path_index < path.len() => path,
            _ => return (false, None),
        };
        let radius_squared = self.lrz_radius * self.lrz_radius;
        let mut start = self.current_state;
        for &node_id in &path[current_path_index..] {
            let end = *self.node(node_id).state();
            let direction = end - start;
            let length = direction.norm();
            let offset = start - self.current_state;

            // Clip to the closed LRZ sphere, including a segment whose endpoints
            // are both outside. Testing only the parent endpoint omitted both
            // the next node's outgoing edge and edges crossing the LRZ boundary.
            let clipped = if length == F::zero() {
                if offset.norm_squared() <= radius_squared { Some((start, end)) } else { None }
            } else if offset.norm_squared() <= radius_squared
                && end.euclidean_distance_squared(&self.current_state) <= radius_squared
            {
                Some((start, end))
            } else {
                let unit = direction / length;
                let projection = -offset.dot(&unit);
                let perpendicular = offset + unit * projection;
                let half_chord_squared = radius_squared - perpendicular.norm_squared();
                if half_chord_squared < F::zero() {
                    None
                } else {
                    let half_chord = half_chord_squared.sqrt();
                    let enter = (projection - half_chord).max(F::zero());
                    let leave = (projection + half_chord).min(length);
                    if enter <= leave {
                        Some((start + unit * enter, start + unit * leave))
                    } else {
                        None
                    }
                }
            };
            if let Some((local_start, local_end)) = clipped {
                if !self.cpr_validity_check_edge(&local_start, &local_end) {
                    return (false, Some(node_id));
                }
            }
            start = end;
        }
        (true, None)
    }

    /// Prunes nodes and edges that are in the CPR.
    fn prune_tree(&mut self) {
        let cpr_obstacles = self.cpr_obstacles.clone();

        let mut pruned_nodes = Vec::new();
        // Prune all nodes within the CPR obstacles
        for obstacle in &cpr_obstacles {
            // Get all nodes within the obstacle's radius
            let nodes_within_radius: Vec<usize> = self
                .nearest_neighbors
                .within_radius(obstacle.center(), obstacle.radius());

            for node_id in nodes_within_radius {
                if self.node(node_id).tree_id == Self::PRUNED_TREE_ID
                    || !obstacle.contains(self.node(node_id).state()) {
                    continue; // Pruned already, or outside the exact capsule.
                }

                debug_assert!(!self.cpr_validity_check_state(self.node(node_id).state()),
                    "Node is within obstacle radius but validity check failed. This should not happen. 
                    Node ID: {}, Obstacle center: {:?}, Obstacle radius: {:?}, Node state: {:?}, Distance: {:?}",
                    node_id, obstacle.center(), obstacle.radius(), self.node(node_id).state(), self.node(node_id).state().euclidean_distance(obstacle.center()),
                );

                // Mark the node as pruned
                self.node_mut(node_id).tree_id = Self::PRUNED_TREE_ID;

                // Store the old parent ID
                let old_parent_id = self.node(node_id).parent;

                // Remove the parent-child relationship
                self.set_node_parent(node_id, None);

                // Set the cumulative cost to infinity
                self.node_mut(node_id).cumulative_cost = F::infinity();

                // Add the node to the pruned nodes list
                pruned_nodes.push((node_id, old_parent_id));
            }
        }

        let mut disjoint_tree_roots = Vec::new();

        // Make the children of the pruned nodes disjoint trees
        for &(node_id, _) in &pruned_nodes {
            // Get the children of the pruned node
            let children = self.node(node_id).children.clone();
            for child_id in children {
                // If the child was pruned, skip it
                if self.node(child_id).tree_id == Self::PRUNED_TREE_ID {
                    continue;
                }

                // Give the child a new tree id
                self.tree_id_counter += 1;
                let new_tree_id = self.tree_id_counter;

                // Update the parent (this also removes the child from the pruned node's children)
                let old_parent_id = self.node(child_id).parent;
                debug_assert!(old_parent_id.is_some() && old_parent_id.unwrap() == node_id);
                self.set_node_parent(child_id, None);

                // Set the cumulative cost of the child to infinity and update the tree id
                let child = self.node_mut(child_id);
                child.cumulative_cost = F::infinity();
                child.tree_id = new_tree_id;

                // propogate info to children
                self.propagate_info_to_children(child_id);

                // Add the child to the disjoint tree roots
                disjoint_tree_roots.push((child_id, old_parent_id));
            }
        }

        // Get all nodes within the obstacle's radius
        let nodes_within_radius: Vec<usize> = self.nearest_neighbors.within_radius(
            &self.current_state,
            self.max_cpr_radius + self.hot_node_neighborhood_radius,
        );

        for node_id in nodes_within_radius {
            if self.node(node_id).tree_id == Self::PRUNED_TREE_ID {
                continue; // The node is already pruned
            }

            // Check if the edge from the parent is valid
            if let Some(parent_id) = self.node(node_id).parent {
                if !self.cpr_validity_check_edge(
                    self.node(parent_id).state(),
                    self.node(node_id).state(),
                ) {
                    // Break the edge
                    self.set_node_parent(node_id, None);
                    self.tree_id_counter += 1;
                    self.node_mut(node_id).tree_id = self.tree_id_counter;
                    self.node_mut(node_id).cumulative_cost = F::infinity();

                    // Propagate info to children
                    self.propagate_info_to_children(node_id);

                    // Add the node to the disjoint tree roots list
                    disjoint_tree_roots.push((node_id, Some(parent_id)));
                }
            }
        }

        // Update the pruned nodes and disjoint tree roots list
        self.pruned_nodes = pruned_nodes;
        self.disjoint_tree_roots = disjoint_tree_roots;
    }

    fn propagate_info_to_children(&mut self, node_id: usize) {
        // Create a stack to manage traversal
        let mut stack = vec![node_id];

        // While there are nodes to process
        while let Some(current_id) = stack.pop() {
            // Get current node's data
            let cumulative_cost = self.node(current_id).cumulative_cost;
            let tree_id = self.node(current_id).tree_id;

            // we should never be propogating pruned nodes
            debug_assert!(tree_id != Self::PRUNED_TREE_ID);

            // Get a copy of the children of the current node
            let children: Vec<usize> = self.node(current_id).children.clone();

            // Update all children and push them onto the stack
            for &child_id in children.iter() {
                self.node_mut(child_id).cumulative_cost =
                    cumulative_cost + self.node(child_id).edge_cost;
                self.node_mut(child_id).tree_id = tree_id;

                // Add child to the stack for processing
                stack.push(child_id);
            }
        }
    }

    /// Backtracks up the tree to find a path to the goal and returns the path as a vector of node IDs.
    /// The caller is responsible for storing the return value in `self.current_path`.
    fn build_planned_path(&mut self) -> Option<Vec<usize>> {
        // get all nodes within the path node radius of current state
        let path_nodes = self
            .nearest_neighbors
            .within_radius(&self.current_state, self.path_node_radius);

        // filter nodes that are not in the main tree
        let path_nodes: Vec<usize> = path_nodes
            .iter()
            .filter(|&&node_id| self.node(node_id).tree_id == Self::MAIN_TREE_ID)
            .copied()
            .collect();

        // filter out nodes where the edge to the node from the current state is not valid
        let mut path_nodes: Vec<usize> = path_nodes
            .iter()
            .filter(|&&node_id| {
                self.cpr_validity_check_edge(&self.current_state, self.node(node_id).state())
                    && self
                        .static_validity_checker
                        .is_edge_valid(&self.current_state, self.node(node_id).state())
            })
            .copied()
            .collect();

        if path_nodes.is_empty() {
            return None; // No nodes are within the path node radius
        }

        if !self.robot_inside_ohzs.is_empty() {
            // The robot is inside an OHZ
            // in this case, we prioritize the path nodes that are not in the OHZ
            let path_nodes_outside_ohz: Vec<usize> = path_nodes
                .iter()
                .filter(|&&node_id| {
                    !self
                        .robot_inside_ohzs
                        .iter()
                        .any(|ohz| ohz.contains(self.node(node_id).state()))
                })
                .copied()
                .collect();
            if !path_nodes_outside_ohz.is_empty() {
                path_nodes = path_nodes_outside_ohz; // Use the nodes outside the OHZ
            }
        }

        // Keep the original cost preference, but do not return a stale parent
        // chain merely because its first connector is clear. Try the next
        // existing candidate when a cheaper chain is invalid under the current
        // static checker; this does not introduce another search algorithm.
        path_nodes.sort_by_key(|&node_id| {
            let node = self.node(node_id);
            OrderedFloat::from(
                self.current_state.euclidean_distance(node.state()) + node.cumulative_cost,
            )
        });
        for best_node_id in path_nodes {
            let mut current_node_id = best_node_id;
            let mut path = Vec::new();
            // A valid parent chain cannot contain more nodes than the arena.
            for _ in 0..self.nodes.len() {
                let node = self.node(current_node_id);
                if node.tree_id != Self::MAIN_TREE_ID
                    || !self.static_validity_checker.is_state_valid(node.state())
                {
                    break;
                }
                path.push(current_node_id);
                if let Some(parent_id) = node.parent {
                    if !self.static_validity_checker.is_edge_valid(
                        node.state(), self.node(parent_id).state())
                    {
                        break;
                    }
                    current_node_id = parent_id;
                } else {
                    if current_node_id == 0
                        && node.state().euclidean_distance(&self.goal) < self.goal_tolerance
                    {
                        return Some(path);
                    }
                    break;
                }
            }
        }
        None
    }

    // Search each spatial shell once until a hot-node is found. The last shell
    // reaches the configured maximum even when multiplication would overshoot.
    fn find_hot_node_with_expanding_lsr(
        &mut self,
        search_center: &RealVectorState<F, N>,
    ) -> Option<HotNode<F>> {
        let mut radius = self.lsr_initial_radius.min(self.lsr_max_radius);
        let mut searched_radius = F::zero();
        loop {
            if let Some(hot) = self.hot_node_search(search_center, radius, searched_radius) {
                return Some(hot);
            }
            if radius >= self.lsr_max_radius { return None; }
            let next = (radius * self.lsr_expansion_factor).min(self.lsr_max_radius);
            if next <= radius { return None; }
            searched_radius = radius;
            radius = next;
        }
    }

    /// Finds the highest-utility hot node within the Local Search Region (LSR).
    /// A zero already-searched radius denotes the first closed ball (including
    /// its center); subsequent calls exclude the previously searched ball.
    ///
    /// A hot node
    ///     - has neighboring nodes that belong to different trees
    ///     - has feasible edges to these neighboring nodes
    ///
    /// Parameters
    /// - `search_center`: The center of the local search region (the position of the closest pruned node).
    /// - `lsr_size`: The size of the Local Search Region (LSR).
    /// - `lsr_already_searched`: The size of the LSR that is already searched.
    ///  
    fn hot_node_search(
        &mut self,
        search_center: &RealVectorState<F, N>,
        lsr_radius: F,
        lsr_already_searched: F,
    ) -> Option<HotNode<F>> {
        // Get all nodes within the LSR radius in sorted order, nearest first
        let nodes_within_lsr: Vec<usize> = self
            .nearest_neighbors
            .within_radius_sorted(search_center, lsr_radius);

        let mut best_hot_node: Option<HotNode<F>> = None;
        for &node_id in &nodes_within_lsr {
            if lsr_already_searched > F::zero()
                && self.node(node_id).state().euclidean_distance(search_center) <= lsr_already_searched
            {
                continue;
            }
            // If the node is pruned, skip it
            if self.node(node_id).tree_id == Self::PRUNED_TREE_ID {
                continue;
            }

            // Check if the node is a hot node
            let node = self.node(node_id);
            let feasible_neighbors = self.get_feasible_neighbors_of_different_tree(node);

            let Some(nearest) = self.nearest_eligible_neighbor(node, &feasible_neighbors) else {
                continue;
            };
            let utility = self.utility(node, self.node(nearest));

            if best_hot_node.as_ref().map_or(true, |best| utility > best.utility
                || (utility == best.utility && node_id < best.node_id)) {
                // Create a new hot node with the current node ID and its feasible neighbors
                best_hot_node = Some(HotNode {
                    node_id,
                    feasible_neighbors_of_different_tree: feasible_neighbors,
                    utility,
                });
            }
        }

        best_hot_node
    }

    fn get_feasible_neighbors_of_different_tree(&self, node: &Node<F, N>) -> Vec<usize> {
        let neighbors = self
            .nearest_neighbors
            .within_radius(node.state(), self.hot_node_neighborhood_radius);
        let mut feasible_neighbors = Vec::new();
        for &neighbor_id in neighbors.iter() {
            if self.node(neighbor_id).tree_id != Self::PRUNED_TREE_ID
                && self.node(neighbor_id).tree_id != node.tree_id
                && self.cpr_validity_check_edge(node.state(), self.node(neighbor_id).state())
                && self
                    .static_validity_checker
                    .is_edge_valid(node.state(), self.node(neighbor_id).state())
            {
                feasible_neighbors.push(neighbor_id);
            }
        }
        feasible_neighbors
    }

    fn nearest_eligible_neighbor(&self, node: &Node<F, N>, neighbors: &[usize]) -> Option<usize> {
        neighbors.iter().copied().filter(|&id| {
            self.node(id).tree_id != Self::PRUNED_TREE_ID
                && self.node(id).tree_id != node.tree_id
        }).min_by_key(|&id| (
            OrderedFloat::from(node.state().euclidean_distance_squared(self.node(id).state())), id))
    }

    // SMART-3D II-C2, Eq. (2): cost goes through the nearest eligible
    // neighbor, using that neighbor's real goal-tree cost or Euclidean estimate.
    fn utility(&self, node: &Node<F, N>, nearest: &Node<F, N>) -> F {
        let state = node.state();
        let distance_to_robot = self.current_state.euclidean_distance(&state);
        let connection_distance = state.euclidean_distance(nearest.state());
        let distance_to_goal = if nearest.tree_id == Self::MAIN_TREE_ID {
            nearest.cumulative_cost
        } else {
            nearest.state().euclidean_distance(&self.goal)
        };
        F::one() / (distance_to_robot + connection_distance + distance_to_goal)
    }

    fn reconnect_best_hot_node(&mut self, hot_node: &HotNode<F>) {
        // II-C3 connects one nearest eligible neighbor, then the outer repair
        // loop recomputes hot-nodes and utilities after the subtree IDs change.
        let node_id = hot_node.node_id;
        if self.node(node_id).tree_id == Self::PRUNED_TREE_ID { return; }
        if let Some(neighbor_id) = self.nearest_eligible_neighbor(
            self.node(node_id), &hot_node.feasible_neighbors_of_different_tree)
        {
            if self.cpr_validity_check_edge(self.node(node_id).state(), self.node(neighbor_id).state())
                && self.static_validity_checker.is_edge_valid(
                    self.node(node_id).state(), self.node(neighbor_id).state())
            {
                self.reconnect_nodes(node_id, neighbor_id);
            }
        }
    }

    fn reconnect_sample_to_neighbors(&mut self, hot_node: &HotNode<F>) {
        // Algorithm 1, standard sampling repair: join the new sample to all
        // nearby reachable subtrees. This differs from informed hot-node repair.
        let node_id = hot_node.node_id;
        let feasible_neighbors = &hot_node.feasible_neighbors_of_different_tree;

        for &neighbor_id in feasible_neighbors.iter() {
            debug_assert!(self.node(node_id).tree_id != Self::PRUNED_TREE_ID);
            debug_assert!(self.node(neighbor_id).tree_id != Self::PRUNED_TREE_ID);

            if self.node(node_id).tree_id == self.node(neighbor_id).tree_id {
                // Skip if they are in same tree, which is possible after a reconnection
                continue;
            }

            self.reconnect_nodes(node_id, neighbor_id);
        }
    }

    /// Reconnects two nodes of different trees.
    /// If any node is the MAIN_TREE, it is guaranteed to be the parent of the other node.
    /// When nodes are reconnected, the child node will be added to set of optimization start nodes.
    fn reconnect_nodes(&mut self, node_a_id: usize, node_b_id: usize) {
        let node_a_tree_id = self.node(node_a_id).tree_id;
        let node_b_tree_id = self.node(node_b_id).tree_id;

        debug_assert!(node_a_tree_id != Self::PRUNED_TREE_ID);
        debug_assert!(node_b_tree_id != Self::PRUNED_TREE_ID);
        debug_assert!(self.debug_check_tree_consistency());
        assert!(node_a_tree_id != node_b_tree_id);

        if node_a_tree_id < node_b_tree_id {
            // The node with the lower tree ID is the parent
            // This guarantees that the MAIN_TREE is always the parent of the other node
            // This also means new sampled nodes will always be added to an existing tree

            // Make node_b the root of its own subtree
            self.make_root_of_subtree(node_b_id);
            debug_assert!(self.debug_check_tree_consistency());

            // Set node_a as the parent of node_b
            self.set_node_parent(node_b_id, Some(node_a_id));

            // check the edge for validity
            debug_assert!(self.cpr_validity_check_edge(
                self.node(node_a_id).state(),
                self.node(node_b_id).state()
            ));

            let edge_cost = self
                .node(node_a_id)
                .state()
                .euclidean_distance(self.node(node_b_id).state());

            let cumulative_cost = self.node(node_a_id).cumulative_cost + edge_cost;

            self.node_mut(node_b_id).tree_id = node_a_tree_id;
            self.node_mut(node_b_id).edge_cost = edge_cost;
            self.node_mut(node_b_id).cumulative_cost = cumulative_cost;

            self.propagate_info_to_children(node_b_id);
            debug_assert!(self.debug_check_tree_consistency());
            self.add_optimization_start_node(node_b_id);
        } else {
            // Make node_a the root of its own subtree
            self.make_root_of_subtree(node_a_id);

            // Set node_b as the parent of node_a
            self.set_node_parent(node_a_id, Some(node_b_id));

            // check the motion for validity
            debug_assert!(self.cpr_validity_check_edge(
                self.node(node_b_id).state(),
                self.node(node_a_id).state()
            ));

            let edge_cost = self
                .node(node_b_id)
                .state()
                .euclidean_distance(self.node(node_a_id).state());

            let cumulative_cost = self.node(node_b_id).cumulative_cost + edge_cost;

            self.node_mut(node_a_id).cumulative_cost = cumulative_cost;
            self.node_mut(node_a_id).tree_id = node_b_tree_id;
            self.node_mut(node_a_id).edge_cost = edge_cost;
            self.node_mut(node_a_id).cumulative_cost = cumulative_cost;
            self.propagate_info_to_children(node_a_id);
            debug_assert!(self.debug_check_tree_consistency());
            self.add_optimization_start_node(node_a_id);
        }

        debug_assert!(self.node(node_a_id).tree_id == self.node(node_b_id).tree_id);
    }

    /// Makes a node the root of its own subtree (all other nodes become its successors).
    /// Can only be used on disjoint subtrees, not on the main tree.
    fn make_root_of_subtree(&mut self, node_id: usize) {
        debug_assert!(self.node(node_id).tree_id != Self::PRUNED_TREE_ID);
        debug_assert!(self.node(node_id).tree_id != Self::MAIN_TREE_ID);

        let mut previous_node_id = None;
        let mut current_node_id = Some(node_id);

        while current_node_id.is_some() {
            let current: usize = current_node_id.unwrap();
            let old_parent_id = self.node(current).parent;

            // Set the parent of the current node to the previous node
            self.set_node_parent(current, previous_node_id);
            if let Some(new_parent) = previous_node_id {
                // Update the edge_cost
                let edge_cost = self
                    .node(new_parent)
                    .state()
                    .euclidean_distance(self.node(current).state());

                debug_assert!(self.node(new_parent).tree_id == self.node(current).tree_id);
                self.node_mut(current).edge_cost = edge_cost;
            }

            // The old parent of the current node becomes the new current node
            previous_node_id = current_node_id;
            current_node_id = old_parent_id;

            // We don't need to update cost as it should be infinite for non-main tree nodes
            debug_assert!(self.node(previous_node_id.unwrap()).cumulative_cost == F::infinity());
        }
    }

    /// Each repair iteration returns to the outer time/iteration budget even
    /// when the domain contains no valid/connectable sample.
    fn sampling_tree_repair_iteration(&mut self) {
        for _ in 0..256 {
            let state = self.sampling_distribution.sample();
            if self.cpr_validity_check_state(&state)
                && self.static_validity_checker.is_state_valid(&state)
                && self.add_new_node_at_state_and_connect(state) { break; }
        }
    }

    /// Adds a new node at the specified state and connects it to an existing tree.
    /// Returns true if the node was added successfully, false otherwise.
    fn add_new_node_at_state_and_connect(&mut self, state: RealVectorState<F, N>) -> bool {
        if self.nodes.len() >= Self::CLEAR_SAMPLING_MAX_ARENA { return false; }
        self.tree_id_counter += 1;
        let new_node = Node::new(
            state.clone(),
            None,
            F::infinity(),
            F::infinity(),
            self.tree_id_counter,
        );

        // get feasible neighbors of different trees
        let neighbors = self.get_feasible_neighbors_of_different_tree(&new_node);
        if neighbors.is_empty() {
            // If there are no feasible neighbors, we can skip adding the node
            return false;
        }

        // add the node to the node list and nearest neighbors data structure
        let new_node_id = self.add_node(new_node);

        let hot_node = HotNode {
            node_id: new_node_id,
            feasible_neighbors_of_different_tree: neighbors.clone(),
            utility: F::zero(), // Utility doesn't matter here
        };

        // Connect the new node to its neighbors
        self.reconnect_sample_to_neighbors(&hot_node);
        true
    }

    /// Adds a node to the tree and the nearest neighbors data structure.
    fn add_node(&mut self, node: Node<F, N>) -> usize {
        let index = self.nodes.len();
        self.nearest_neighbors.add(node.state().clone(), index);
        self.nodes.push(node);
        index
    }

    fn add_optimization_start_node(&mut self, node_id: usize) {
        if self.node(node_id).tree_id != Self::MAIN_TREE_ID {
            return;
        }

        // Check if a parent of the node is already in the optimization start nodes
        let mut parent_id = node_id;
        while let Some(parent) = self.node(parent_id).parent {
            if self.optimization_start_nodes.contains(&parent) {
                return;
            }
            parent_id = parent;
        }

        self.optimization_start_nodes.push(node_id);
    }

    fn tree_optimization(&mut self) {
        // Take ownership of the optimization_start_nodes, leaving it empty
        let optimization_start_nodes = std::mem::take(&mut self.optimization_start_nodes);

        // Sort the nodes by their cumulative cost
        let mut sorted_nodes = optimization_start_nodes;
        sorted_nodes.sort_by(|&a, &b| {
            self.node(a)
                .cumulative_cost
                .partial_cmp(&self.node(b).cumulative_cost)
                .unwrap()
        });

        // Iterate over the sorted nodes and perform the rewiring
        for node_id in sorted_nodes {
            self.rewire_cascade(node_id);
        }

        // `self.optimization_start_nodes` remains empty after this method
        debug_assert!(self.optimization_start_nodes.is_empty());
    }

    fn rewire_cascade(&mut self, node_id: usize) {
        // Although this is not the most technically correct way to implement a rewiring cascade,
        // this is the same way as it was done in the original SMART-2D implementation.
        // Original implementation: https://github.com/ZongyuanShen/SMART/blob/main/SMART/SMART.cpp#L1225
        // This uses a depth-first search, but a queue based approach would be more correct.
        self.rewire_to_best_parent(node_id);
        let children = self.node(node_id).children.clone();
        for child_id in children {
            self.rewire_cascade(child_id);
        }
    }

    /// Rewires the node to the best parent within the hot nod rewiring radius.
    fn rewire_to_best_parent(&mut self, node_id: usize) {
        let neighbors = self.nearest_neighbors.within_radius(
            self.node(node_id).state(),
            self.hot_node_neighborhood_radius,
        );

        let mut best_parent = self.node(node_id).parent;
        let mut best_cost = self.node(node_id).cumulative_cost;
        let mut best_edge_cost = self.node(node_id).edge_cost;

        for &neighbor_id in neighbors.iter() {
            if Some(neighbor_id) == best_parent || neighbor_id == node_id {
                continue;
            }
            let neighbor = self.node(neighbor_id);

            // Check if the neighbor is in the same tree or if it is prunedl

            let edge_cost = self
                .node(neighbor_id)
                .state()
                .euclidean_distance(self.node(node_id).state());
            let cost = neighbor.cumulative_cost + edge_cost;
            if cost < best_cost
                && self.cpr_validity_check_edge(neighbor.state(), self.node(node_id).state())
                && self
                    .static_validity_checker
                    .is_edge_valid(neighbor.state(), self.node(node_id).state())
            {
                best_parent = Some(neighbor_id);
                best_cost = cost;
                best_edge_cost = edge_cost;
            }
        }

        self.set_node_parent(node_id, best_parent);
        self.node_mut(node_id).cumulative_cost = best_cost;
        self.node_mut(node_id).edge_cost = best_edge_cost;
        self.propagate_info_to_children(node_id);
    }

    fn form_single_tree(&mut self) {
        // Take ownership of the pruned nodes, leaving it empty
        let pruned = std::mem::take(&mut self.pruned_nodes);

        // Iterate over the pruned nodes and add them back to the tree
        for &(node_id, old_parent_id) in pruned.iter() {
            // The pruned nodes should have no parent and a pruned tree id
            assert!(self.node(node_id).tree_id == Self::PRUNED_TREE_ID);
            assert!(self.node(node_id).parent.is_none());

            // Permanent static-invalid nodes/edges must never be resurrected.
            if !self.static_validity_checker.is_state_valid(self.node(node_id).state()) {
                continue;
            }
            let old_parent_id = old_parent_id.filter(|&parent| {
                self.static_validity_checker.is_state_valid(self.node(parent).state())
                    && self.static_validity_checker.is_edge_valid(
                        self.node(parent).state(), self.node(node_id).state())
            });
            self.set_node_parent(node_id, old_parent_id);

            if old_parent_id.is_none() {
                // A static update may have left a valid, non-goal forest root.
                if node_id == 0 {
                    self.node_mut(node_id).tree_id = Self::MAIN_TREE_ID;
                    self.node_mut(node_id).cumulative_cost = F::zero();
                    self.node_mut(node_id).edge_cost = F::zero();
                } else {
                    self.tree_id_counter += 1;
                    self.node_mut(node_id).tree_id = self.tree_id_counter;
                    self.node_mut(node_id).cumulative_cost = F::infinity();
                    self.node_mut(node_id).edge_cost = F::infinity();
                }
            } else {
                let old_parent_id = old_parent_id.unwrap();

                if self.node(old_parent_id).tree_id == Self::PRUNED_TREE_ID {
                    // the parent is also pruned
                    // this node will be handled by propogation from the parent
                    assert!(pruned.iter().any(|&(n, _)| n == old_parent_id));
                    continue;
                }

                let edge_cost = self
                    .node(old_parent_id)
                    .state()
                    .euclidean_distance(self.node(node_id).state());

                // We need to fix the cumulative cost
                self.node_mut(node_id).cumulative_cost =
                    self.node(old_parent_id).cumulative_cost + edge_cost;
                self.node_mut(node_id).edge_cost = edge_cost;

                // The tree id of the node should be the same as its parent
                self.node_mut(node_id).tree_id = self.node(old_parent_id).tree_id;
            }

            // Propagate the information to the children (if any)
            self.propagate_info_to_children(node_id);
        }

        debug_assert!(self.debug_check_tree_consistency());
        debug_assert!(self.debug_check_costs());

        // Take ownership of the disjoint tree roots, leaving it empty
        let disjoint_tree_roots = std::mem::take(&mut self.disjoint_tree_roots);

        // Iterate over the disjoint tree roots and reconnect them to the main tree
        for &(node_id, old_parent_id) in disjoint_tree_roots.iter() {
            let Some(old_parent_id) = old_parent_id else { continue; };
            if self.node(old_parent_id).tree_id == Self::PRUNED_TREE_ID
                || !self.static_validity_checker.is_state_valid(self.node(node_id).state())
                || !self.static_validity_checker.is_edge_valid(
                    self.node(old_parent_id).state(), self.node(node_id).state())
            {
                continue;
            }

            // If the node is already in the main tree, skip it
            if self.node(node_id).tree_id == Self::MAIN_TREE_ID {
                continue;
            }

            // If this already has the same tree id as the old parent, don't reconnect
            if self.node(old_parent_id).tree_id == self.node(node_id).tree_id {
                continue;
            }

            // Make the node the root of its own subtree
            self.make_root_of_subtree(node_id);

            // Set the node back to its old parent
            self.set_node_parent(node_id, Some(old_parent_id));

            // Set the edge and cumulative cost
            let edge_cost = self
                .node(old_parent_id)
                .state()
                .euclidean_distance(self.node(node_id).state());
            self.node_mut(node_id).edge_cost = edge_cost;
            self.node_mut(node_id).cumulative_cost =
                self.node(old_parent_id).cumulative_cost + self.node(node_id).edge_cost;

            // The tree id of the node should be the same as its parent
            self.node_mut(node_id).tree_id = self.node(old_parent_id).tree_id;

            // Propagate the information to the children
            self.propagate_info_to_children(node_id);
        }

        // pruned nodes and disjoint tree roots should be empty after this method
        debug_assert!(self.pruned_nodes.is_empty());
        debug_assert!(self.disjoint_tree_roots.is_empty());
    }

    /// Check if a state is a valid, considering the CPR obstacles.
    fn cpr_validity_check_state(&self, state: &RealVectorState<F, N>) -> bool {
        if state.euclidean_distance(&self.current_state) > self.max_cpr_radius {
            return true; // The state is outside the CPR, so we don't need to check for collisions
        }

        for obs in &self.cpr_obstacles {
            if obs.contains(state) {
                return false; // The state is in collision with a CPR obstacle
            }
        }

        true // The state is valid (not in collision with any CPR obstacle)
    }

    /// Check if an edge is valid, considering the CPR obstacles.
    fn cpr_validity_check_edge(
        &self,
        start: &RealVectorState<F, N>,
        end: &RealVectorState<F, N>,
    ) -> bool {
        // A long edge can cross CPR with both endpoints outside it. Use its
        // nearest point for this broadphase, not the nearer endpoint.
        let direction = *end - *start;
        let squared_length = direction.norm_squared();
        let fraction = if squared_length > F::zero() {
            ((self.current_state - *start).dot(&direction) / squared_length)
                .max(F::zero()).min(F::one())
        } else { F::zero() };
        let nearest = *start + direction * fraction;
        if self.current_state.euclidean_distance(&nearest) > self.max_cpr_radius + self.hot_node_neighborhood_radius {
            return true; // The edge is outside the CPR, so we don't need to check for collisions
        }

        for obs in &self.cpr_obstacles {
            if obs.intersects_edge(start, end) {
                return false; // The edge is in collision with a CPR obstacle
            }
        }

        true // The edge is valid (not in collision with any CPR obstacle)
    }

    /// Sets the parent of a node, including by updating the new and old parent's child list.
    fn set_node_parent(&mut self, node_id: usize, parent: Option<usize>) {
        // Remove the node from the old parent's child list
        let old_parent_id = self.node(node_id).parent;
        if let Some(old_parent_id) = old_parent_id {
            self.node_mut(old_parent_id).remove_child(node_id);
        }

        // Set the new parent and add the node to the parent's child list
        self.node_mut(node_id).parent = parent;
        if let Some(parent_id) = parent {
            self.node_mut(parent_id).add_child(node_id);
        }
    }

    fn node(&self, node_id: usize) -> &Node<F, N> {
        &self.nodes[node_id]
    }

    fn node_mut(&mut self, node_id: usize) -> &mut Node<F, N> {
        &mut self.nodes[node_id]
    }

    // Debugging Assertion Functions
    // -----------------------------

    /// Checks the consistency of the tree.
    /// This checks the following:
    /// 1. Parent consistency: The parent of a node should have the node as a child.
    /// 2. Child consistency: The children of a node should have the node as a parent.
    /// 3. Cycle detection: There should be no cycles in the tree.
    /// 4. Tree ID consistency: The tree ID of a node should be the same as its parent.
    fn debug_check_tree_consistency(&self) -> bool {
        for (index, node) in self.get_nodes().iter().enumerate() {
            // Check every node for parent consistency.
            // This means that the parent of a node should have the node as a child.
            if let Some(parent_index) = node.parent {
                let parent = self.node(parent_index);
                if !parent.children.contains(&index) {
                    println!(
                        "Parent inconsistency! Node {} has parent {} but parent does not have child {}",
                        index, parent_index, index
                    );
                    return false;
                }
                if parent.tree_id != node.tree_id {
                    println!(
                        "Tree ID inconsistency! Node {} has tree id {} and parent {} with tree ID {}",
                        index, node.tree_id, parent_index, parent.tree_id
                    );
                    return false;
                }
            }

            // Check every node for child consistency.
            // This means that the children of a node should have the node as a parent.
            for &child_index in &node.children {
                let child = self.node(child_index);
                if child.parent != Some(index) {
                    println!(
                        "Child inconsistency! Node {} has child {} but child has parent {:?}",
                        index, child_index, child.parent
                    );
                    return false;
                }
            }

            // Check every node for cycles
            let mut seen = std::collections::HashSet::new();
            let mut current_node_id = index;
            while let Some(parent_id) = self.node(current_node_id).parent {
                if current_node_id < index {
                    // We know we already did a cycle check starting from this node
                    break;
                }
                if !seen.insert(current_node_id) {
                    println!(
                        "Cycle detected in tree! Node {} is visited twice",
                        current_node_id
                    );
                    return false;
                }
                current_node_id = parent_id;
            }
        }

        true
    }

    /// Checks that nodes were pruned correctly.
    /// This checks the following:
    /// 1. At least one node was pruned, or at least one disjoint tree root was created.   
    /// 2. All pruned nodes should have a tree ID of PRUNED_TREE_ID.
    /// 3. All pruned nodes are within the max pruning radius.
    /// 4. All pruned nodes are not valid.
    /// 5. All non-pruned nodes within the LRZ should be safe.
    /// 6. All disjoint tree roots should have unique tree IDs.
    /// 7. All disjoint tree roots should have a tree ID greater than MAIN_TREE_ID.
    /// 8. All disjoint tree roots should have no parent.
    fn debug_check_pruning(&self) -> bool {
        // No new cuts are necessary when retrying an already disconnected
        // permanent forest or when only the robot-to-path connector was blocked.

        // all pruned nodes should have tree id of PRUNED_TREE_ID
        for &(node_id, _) in self.pruned_nodes.iter() {
            debug_assert!(self.node(node_id).tree_id == Self::PRUNED_TREE_ID);
            debug_assert!(!self.cpr_validity_check_state(self.node(node_id).state()));
        }

        // check all non-pruned nodes are safe
        if !self.debug_check_non_pruned_nodes_are_safe() {
            println!("Pruned nodes list: {:?}", self.pruned_nodes);
            return false;
        }

        let mut disjoint_tree_roots = std::collections::HashSet::new();
        for (node_id, _) in self.disjoint_tree_roots.iter() {
            // all disjoint tree ids are unique
            debug_assert!(disjoint_tree_roots.insert(self.node(*node_id).tree_id));
            // all disjoint tree roots are greater than main tree id
            debug_assert!(self.node(*node_id).tree_id > Self::MAIN_TREE_ID);
            // all disjoint tree roots have no parent
            debug_assert!(self.node(*node_id).parent.is_none());
        }

        true
    }

    /// Checks that all nodes that are not pruned are safe.
    fn debug_check_non_pruned_nodes_are_safe(&self) -> bool {
        // check all non-pruned nodes are safe
        for node_id in 0..self.get_nodes().len() {
            if self.node(node_id).tree_id == Self::PRUNED_TREE_ID {
                continue;
            }
            if !self.cpr_validity_check_state(self.node(node_id).state()) {
                println!(
                    "Node {} is not safe! State in collision with dynamic CPR obstacle.",
                    node_id
                );
                println!("Node {} state: {:?}", node_id, self.node(node_id).state());
                println!("Obstacle list: {:?}", self.cpr_obstacles);
                println!("CPR max radius: {:?}", self.max_cpr_radius);
                println!("Current state: {:?}", self.current_state);
                println!("LRZ radius: {:?}", self.lrz_radius);
                return false;
            }
            if !self
                .static_validity_checker
                .is_state_valid(self.node(node_id).state())
            {
                println!(
                    "Node {} is not safe! State in collision with static obstacle.",
                    node_id
                );
                return false;
            }

            if let Some(parent) = self.node(node_id).parent {
                if !self
                    .cpr_validity_check_edge(self.node(parent).state(), self.node(node_id).state())
                {
                    println!(
                        "Node {} is not safe! Edge from parent {} is in collision with dynamic CPR obstacle.",
                        node_id, parent
                    );
                    if self.disjoint_tree_roots.contains(&(node_id, Some(parent))) {
                        println!("Node {} is a disjoint tree root.", node_id);
                    }
                    println!("Node {} state: {:?}", node_id, self.node(node_id).state());
                    println!("Parent {} state: {:?}", parent, self.node(parent).state());
                    println!("Obstacle list: {:?}", self.cpr_obstacles);
                    println!("CPR max radius: {:?}", self.max_cpr_radius);
                    println!("Current state: {:?}", self.current_state);
                    println!("LRZ radius: {:?}", self.lrz_radius);
                    return false;
                }
                if !self
                    .static_validity_checker
                    .is_edge_valid(self.node(parent).state(), self.node(node_id).state())
                {
                    println!(
                        "Node {} is not safe! Edge from parent {} is in collision with static obstacle.",
                        node_id, parent
                    );
                    return false;
                }
            }
        }
        true
    }

    /// Finalized repair may retain permanently invalid vertices and disconnected
    /// valid subtrees. Only the component rooted at goal node zero is MAIN_TREE.
    fn debug_check_finalized_forest(&self) -> bool {
        let mut roots = std::collections::HashSet::new();
        for (id, node) in self.nodes.iter().enumerate() {
            if node.tree_id == Self::PRUNED_TREE_ID {
                if node.parent.is_some() || !node.children.is_empty() { return false; }
            } else if node.parent.is_none() {
                if (id == 0 && node.tree_id != Self::MAIN_TREE_ID)
                    || (id != 0 && node.tree_id <= Self::MAIN_TREE_ID)
                    || !roots.insert(node.tree_id)
                { return false; }
            } else if node.tree_id < Self::MAIN_TREE_ID {
                return false;
            }
        }
        self.debug_check_tree_consistency() && self.debug_check_costs()
    }

    /// Checks that every node has a tree_id of MAIN_TREE_ID.
    /// If not, prints a summary of how many nodes are in each other tree.
    fn debug_check_all_nodes_in_main_tree(&self) -> bool {
        let mut counts: HashMap<i32, usize> = HashMap::new();

        for node in self.get_nodes().iter() {
            if node.tree_id != Self::MAIN_TREE_ID {
                *counts.entry(node.tree_id).or_insert(0) += 1;
            }
        }

        if counts.is_empty() {
            // All nodes are in the main tree
            true
        } else {
            println!(
                "Debug check failed: found {} nodes outside main tree (id {})",
                counts.values().sum::<usize>(),
                Self::MAIN_TREE_ID
            );
            for (tree_id, &count) in &counts {
                println!(" - {} node(s) in tree_id {}", count, tree_id);
            }
            false
        }
    }

    /// Checks that all cumulative node costs match edge costs for nodes in the main tree.
    fn debug_check_costs(&self) -> bool {
        for node_id in 0..self.get_nodes().len() {
            let node = self.node(node_id);

            if node.tree_id != Self::MAIN_TREE_ID {
                continue;
            }

            if node.parent.is_none() {
                continue; // The root node has no parent, so we skip it
            }

            let parent_id = node.parent.unwrap();
            let parent = self.node(parent_id);

            // cumulative_cost increases away from root: node.cum = parent.cum + edge_cost
            let expected_edge_cost = node.cumulative_cost - parent.cumulative_cost;
            if (expected_edge_cost - node.edge_cost).abs() > F::from(0.01).unwrap() {
                println!(
                    "Cost mismatch for node {}: expected edge cost {}, but got {}",
                    node_id,
                    expected_edge_cost.to_f32().unwrap(),
                    node.edge_cost.to_f32().unwrap()
                );
                return false;
            }
        }
        true
    }
}

#[cfg(test)]
#[path = "path_safety_tests.rs"]
mod path_safety_tests;

#[cfg(test)]
#[path = "static_forest_tests.rs"]
mod static_forest_tests;

#[cfg(test)]
#[path = "paper_hotnode_tests.rs"]
mod paper_hotnode_tests;
