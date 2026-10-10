use crate::rrt::neighbors::NearestNeighbors;
use crate::rrt::sampling::SamplingDistribution;
use crate::rrt::state::RealVectorState;
use crate::rrt::termination::TerminationCondition;
use crate::rrt::validity_checker::ValidityChecker;
use num_traits::Float;

/// A node in the RRT tree.
#[derive(Clone)]
pub struct Node<F: Float, const N: usize> {
    /// The state in N-dimensional space.
    state: RealVectorState<F, N>,
    /// The index of the parent node (None if the node is the root).
    parent: Option<usize>,
}

impl<F: Float, const N: usize> Node<F, N> {
    /// Constructs a new node.
    /// Parameters:
    /// - `state`: The state in N-dimensional space.
    /// - `parent`: The index of the parent node (None if the node is the root).
    pub fn new(state: RealVectorState<F, N>, parent: Option<usize>) -> Self {
        Self { state, parent }
    }

    pub fn state(&self) -> &RealVectorState<F, N> {
        &self.state
    }

    pub fn parent(&self) -> Option<usize> {
        self.parent
    }
}

/// A Rapidly-exploring Random Tree (RRT) planner.
/// Template Parameters:
/// - `F`: The floating-point type.
/// - `N`: The dimension of the space.
/// - `NN`: The nearest neighbors data structure.
pub struct RRT<F: Float, const N: usize, NN: NearestNeighbors<F, N>> {
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
}

impl<F: Float, const N: usize, NN: NearestNeighbors<F, N>> RRT<F, N, NN> {
    /// Constructs a new RRT planner.
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
        };
        let root_node = Node::new(root, None);
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

    fn steer(
        &self,
        from: &RealVectorState<F, N>,
        to: &RealVectorState<F, N>,
    ) -> RealVectorState<F, N> {
        let direction = to - from;
        let distance = direction.norm();
        if distance > self.steering_range {
            let scaled_direction = direction / distance * self.steering_range;
            from + &scaled_direction
        } else {
            to.clone()
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
        let new_state: RealVectorState<F, N> = self.steer(nearest_state, &sample);

        // If the new point or edge is invalid, return.
        if !self.validity_checker.is_state_valid(&new_state)
            || !self
                .validity_checker
                .is_edge_valid(nearest_state, &new_state)
        {
            return;
        }

        // Add the new node to as a child of the nearest node.
        let new_node = Node::new(new_state, Some(nearest_node_index));
        let new_node_index = self.add_node(new_node);

        // If the goal is reached, update the solution node.
        let dist_squared = new_state.euclidean_distance_squared(&self.goal);
        if dist_squared <= self.goal_tolerance * self.goal_tolerance {
            self.solution = Some(new_node_index);
        }
    }

    /// Adds a node to the tree and the nearest neighbors data structure.
    fn add_node(&mut self, node: Node<F, N>) -> usize {
        let index = self.nodes.len();
        self.nearest_neighbors.add(node.state().clone(), index);
        self.nodes.push(node);
        index
    }

    /// Deconstructs the RRT planner into its main components.
    /// This enables the underlying data structures to be accessed without cloning them.
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
