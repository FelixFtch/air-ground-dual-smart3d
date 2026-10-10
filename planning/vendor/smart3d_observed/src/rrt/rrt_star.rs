use crate::rrt::neighbors::NearestNeighbors;
use crate::rrt::sampling::SamplingDistribution;
use crate::rrt::state::RealVectorState;
use crate::rrt::termination::TerminationCondition;
use crate::rrt::validity_checker::ValidityChecker;
use num_traits::Float;

/// A node in the RRT* tree.
#[derive(Clone)]
pub struct Node<F: Float, const N: usize> {
    /// The state in N-dimensional space.
    state: RealVectorState<F, N>,
    /// The index of the parent node (None if the node is the root).
    parent: Option<usize>,
    /// Cost of the path from the root to this node.
    cumulative_cost: F,
}

impl<F: Float, const N: usize> Node<F, N> {
    /// Constructs a new node.
    /// Parameters:
    /// - `state`: The state in N-dimensional space.
    /// - `parent`: The index of the parent node (None if the node is the root).
    pub fn new(state: RealVectorState<F, N>, parent: Option<usize>, cumulative_cost: F) -> Self {
        Self {
            state,
            parent,
            cumulative_cost,
        }
    }

    pub fn state(&self) -> &RealVectorState<F, N> {
        &self.state
    }

    pub fn parent(&self) -> Option<usize> {
        self.parent
    }

    pub fn cumulative_cost(&self) -> F {
        self.cumulative_cost
    }
}

/// A Rapidly-exploring Random Tree (RRT) planner.
/// Template Parameters:
/// - `F`: The floating-point type.
/// - `N`: The dimension of the space.
/// - `NN`: The nearest neighbors data structure.
pub struct RRTstar<F: Float, const N: usize, NN: NearestNeighbors<F, N>> {
    /// The goal state.
    goal: RealVectorState<F, N>,
    /// The tolerance for reaching the goal.
    goal_tolerance: F,
    /// The nodes in the tree.
    nodes: Vec<Node<F, N>>,
    /// Index of the solution node (None if no solution has been found).
    solution: Option<usize>,
    validity_checker: Box<dyn ValidityChecker<F, N>>,
    sampling_distribution: Box<dyn SamplingDistribution<F, N>>,
    nearest_neighbors: NN,
    /// The steering range (maximum distance to steer towards the sample state).
    steering_range: F,
    /// Max connection radius when rewiring the tree.
    max_connection_radius: F,
    gamma: F,
}

impl<F: Float, const N: usize, NN: NearestNeighbors<F, N>> RRTstar<F, N, NN> {
    /// Constructs a new RRTstar planner.
    ///
    /// Parameters:
    /// - `root`: The root of the tree (start state).
    /// - `goal`: The goal state which the tree will try to reach.
    /// - `goal_tolerance`: The tolerance for reaching the goal.
    /// - `validity_checker`: Checks if the edges or nodes as valid.
    /// - `sampling_distribution`: The sampling distribution.
    /// - `steering_range`: The maximum distance to steer towards the sample state.
    /// Returns the RRT planner.
    pub fn new(
        root: RealVectorState<F, N>,
        goal: RealVectorState<F, N>,
        goal_tolerance: F,
        validity_checker: Box<dyn ValidityChecker<F, N>>,
        sampling_distribution: Box<dyn SamplingDistribution<F, N>>,
        steering_range: F,
        max_connection_radius: F,
        gamma: F,
    ) -> Self {
        let mut rrt = Self {
            goal,
            goal_tolerance,
            solution: None,
            nodes: Vec::new(),
            validity_checker,
            sampling_distribution,
            nearest_neighbors: NN::new(),
            steering_range,
            max_connection_radius,
            gamma,
        };
        let root_node = Node::new(root, None, F::zero());
        rrt.add_node(root_node);
        rrt
    }

    /// Attempts to find a solution.
    ///
    /// Terminates and returns true when a solution is found. Otherwise, returns false if the termination condition is met.
    ///
    /// Parameters:
    /// - `termination`: The termination condition.
    pub fn plan_until_solved_or<T: TerminationCondition>(&mut self, termination: &mut T) -> bool {
        loop {
            if termination.evaluate() {
                return false;
            }
            self.iteration();
            if self.solved() {
                return true;
            }
        }
    }

    /// Attempts to find a solution.
    ///
    /// Terminates when the termination condition is met.
    ///
    /// Parameters:
    /// - `termination`: The termination condition.
    pub fn plan_until<T: TerminationCondition>(&mut self, termination: &mut T) {
        while !termination.evaluate() {
            self.iteration();
        }
    }

    /// Run a fixed number of iterations of the RRT algorithm. Does not terminate early if a solution is found.
    ///
    /// Returns true if the RRT found a solution.
    ///
    /// Parameters:
    /// - `iterations`: The number of iterations to run.
    pub fn run_iterations(&mut self, iterations: u32) -> bool {
        for _ in 0..iterations {
            self.iteration();
            if self.solved() {
                return true;
            }
        }
        return false;
    }

    /// Returns true if a solution was found.
    pub fn solved(&self) -> bool {
        self.solution.is_some()
    }

    /// Returns the path from the start to the goal, if a solution was found.
    pub fn get_path(&self) -> Option<Vec<RealVectorState<F, N>>> {
        if !self.solved() {
            return None;
        }

        let mut path = Vec::new();
        let mut current_index = self.solution.unwrap();

        // Reconstruct the path by backtracking up the tree (following the parent pointers).
        while let Some(parent_index) = self.nodes[current_index].parent {
            path.push(self.nodes[current_index].state);
            current_index = parent_index;
        }
        path.push(self.nodes[current_index].state);

        // Reverse the path so that it goes from the start to the goal.
        path.reverse();
        Some(path)
    }

    /// Returns the vector of nodes in the tree.
    pub fn get_tree(&self) -> &Vec<Node<F, N>> {
        &self.nodes
    }

    /// Steers from one state towards another to generate a new state to add to the tree.
    /// Returns the new state and the distance to the target state.
    fn steer(
        &self,
        from: &RealVectorState<F, N>,
        to: &RealVectorState<F, N>,
    ) -> (RealVectorState<F, N>, F) {
        let direction = to - from;
        let distance = direction.norm();
        if distance > self.steering_range {
            let scaled_direction = direction / distance * self.steering_range;
            (from + &scaled_direction, self.steering_range)
        } else {
            (to.clone(), distance)
        }
    }

    /// Expands the tree by one iteration.
    ///
    /// Each iteration of the RRT algorithm consists of the following steps:
    /// 1. Sample a state from the sampling distribution.
    /// 2. Find the nearest node in the tree to the sample state.
    /// 3. Steer the nearest node towards the sample state.
    /// 4. Add the new node to as a child of the nearest node if the edge is valid.
    /// 5. If the goal is reached, update the solution node.
    fn iteration(&mut self) {
        // Sample a state from the sampling distribution.
        let sample = self.sampling_distribution.sample();

        // Find the nearest node in the tree to the sample state.
        let nearest_node_index = self.nearest_neighbors.nearest_one(&sample).unwrap();
        let nearest_state = &self.nodes[nearest_node_index].state;

        // Steer the nearest node towards the sample state to get a new state.
        let (new_state, edge_cost) = self.steer(nearest_state, &sample);

        // If the new point or edge is invalid, return.
        if !self.validity_checker.is_state_valid(&new_state)
            || !self
                .validity_checker
                .is_edge_valid(nearest_state, &new_state)
        {
            return;
        }

        // Create the new node as child of the nearest node.
        let cumulative_cost = self.nodes[nearest_node_index].cumulative_cost + edge_cost;
        let new_node = Node::new(new_state, Some(nearest_node_index), cumulative_cost);

        // Find the neighbors of the new node.
        let neighbors = self
            .nearest_neighbors
            .within_radius(&new_node.state(), self.rewiring_radius());

        // Rewire the new node to the best parent and add to the tree.
        let new_node = self.rewire_to_best_parent(new_node, &neighbors);
        let new_node_index = self.add_node(new_node);

        // Rewire the neighbors of the new node.
        self.rewire_neighbors(new_node_index, &neighbors);

        let dist_squared = new_state.euclidean_distance_squared(&self.goal);
        if dist_squared > self.goal_tolerance * self.goal_tolerance {
            // This node is not the goal.
            return;
        }

        // The goal is reached, update the solution node if the cost is lower.
        if self.solution.is_none()
            || self.nodes[new_node_index].cumulative_cost
                < self.nodes[self.solution.unwrap()].cumulative_cost
        {
            self.solution = Some(new_node_index);
        }
    }

    /// Find the best parent for the new node and rewire it to the new node.
    fn rewire_to_best_parent(&mut self, node: Node<F, N>, neighbors: &Vec<usize>) -> Node<F, N> {
        let mut node = node;
        for &neighbor_index in neighbors {
            if node.parent.is_some() && neighbor_index == node.parent.unwrap() {
                // Skip node which is already the parent.
                continue;
            }

            let neighbor = &self.nodes[neighbor_index];
            let neighbor_state = neighbor.state();
            if self
                .validity_checker
                .is_edge_valid(neighbor_state, &node.state)
            {
                let edge_cost = node.state.euclidean_distance(&neighbor_state);
                let cumulative_cost = neighbor.cumulative_cost + edge_cost;
                if cumulative_cost < node.cumulative_cost {
                    node = Node::new(node.state, Some(neighbor_index), cumulative_cost);
                }
            }
        }
        return node;
    }

    /// Checks if the new node is a better parent for the neighbors of the new node.
    fn rewire_neighbors(&mut self, node_index: usize, neighbors: &Vec<usize>) {
        for &neighbor_index in neighbors {
            let neighbor = &self.nodes[neighbor_index];
            let neighbor_state = neighbor.state();

            if self
                .validity_checker
                .is_edge_valid(&self.nodes[node_index].state(), neighbor_state)
            {
                let edge_cost = self.nodes[node_index]
                    .state()
                    .euclidean_distance(neighbor_state);
                let cumulative_cost = self.nodes[node_index].cumulative_cost + edge_cost;
                if cumulative_cost < neighbor.cumulative_cost {
                    self.nodes[neighbor_index].parent = Some(node_index);
                    self.nodes[neighbor_index].cumulative_cost = cumulative_cost;
                }
            }
        }
    }

    /// Adds a node to the tree and the nearest neighbors data structure.
    fn add_node(&mut self, node: Node<F, N>) -> usize {
        let index = self.nodes.len();
        self.nearest_neighbors.add(node.state().clone(), index);
        self.nodes.push(node);
        index
    }

    fn rewiring_radius(&self) -> F {
        let card_v = F::from(self.nodes.len()).unwrap();
        let d = F::from(N).unwrap();
        let radius = self.gamma * (card_v.ln() / card_v).powf(F::one() / d);
        if radius > self.max_connection_radius {
            self.max_connection_radius
        } else {
            radius
        }
    }

    /// Deconstructs the RRT* planner into its components.
    ///  This enables the underlying data structures to be accessed without cloning them.
    ///
    /// Returns a tuple containing:
    /// - A vector of nodes in the tree.
    /// - The nearest neighbors data structure.
    /// - The validity checker.
    /// - The sampling distribution.
    pub fn deconstruct_into_components(
        self,
    ) -> (
        Vec<Node<F, N>>,
        NN,
        Box<dyn ValidityChecker<F, N>>,
        Box<dyn SamplingDistribution<F, N>>,
    ) {
        (
            self.nodes,
            self.nearest_neighbors,
            self.validity_checker,
            self.sampling_distribution,
        )
    }
}

/// Computes gamma value to achieve asymptotic optimality for the RRT* algorithm.
/// Parameters:
/// - `free_space_volume`: The volume of the free space.
/// - `dimension`: The dimension of the state space.
///
/// Returns:
/// The optimal gamma value.
pub fn optimal_gamma(free_space_volume: f32, dimension: usize) -> f32 {
    if free_space_volume <= 0.0 {
        panic!("The free space volume must be positive.");
    }

    if dimension == 0 {
        panic!("The dimension must be positive.");
    }

    let unit_ball_volume = (std::f32::consts::PI.powf((dimension as f32) / 2.0))
        / special::Gamma::gamma(1.0 + (dimension as f32) / 2.0);

    let gamma = (2.0 * (1.0 + 1.0 / dimension as f32) * free_space_volume / unit_ball_volume)
        .powf(1.0 / dimension as f32);

    gamma
}
