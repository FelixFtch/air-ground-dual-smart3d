use crate::rrt::state::RealVectorState;
use num_traits::Float;

/// Checks if a state or edge is valid (i.e., not in collision).
pub trait ValidityChecker<F: Float, const N: usize> {
    /// Checks if a state is valid (i.e., does not collide with obstacles).
    ///
    /// Parameters:
    /// - `state`: The state to check.
    ///
    /// Returns:
    /// Whether the state is valid.
    fn is_state_valid(&self, state: &RealVectorState<F, N>) -> bool;

    /// Checks if an edge is valid (i.e., does not collide with obstacles).
    ///
    /// Parameters:
    /// - `a`: The start point of the edge.
    /// - `b`: The end point of the edge.
    ///
    /// Returns:
    /// Whether the edge is valid.
    fn is_edge_valid(&self, a: &RealVectorState<F, N>, b: &RealVectorState<F, N>) -> bool;
}

/// A simple validity checker that always returns true (i.e., all points and edges are valid).
pub struct AlwaysValid<F: Float, const N: usize> {
    _phantom: std::marker::PhantomData<F>,
}

impl<F: Float, const N: usize> AlwaysValid<F, N> {
    /// Constructs a new AlwaysValid.
    pub fn new() -> Self {
        Self {
            _phantom: std::marker::PhantomData,
        }
    }
}

impl<F: Float, const N: usize> ValidityChecker<F, N> for AlwaysValid<F, N> {
    fn is_state_valid(&self, _point: &RealVectorState<F, N>) -> bool {
        true
    }

    fn is_edge_valid(&self, _a: &RealVectorState<F, N>, _b: &RealVectorState<F, N>) -> bool {
        true
    }
}

/// A validity checker that takes the union of multiple validity checkers.
/// If any of the checkers return false, the point or edge is considered invalid.
pub struct UnionValidityChecker<F: Float, const N: usize> {
    checkers: Vec<Box<dyn ValidityChecker<F, N>>>,
}

impl<F: Float, const N: usize> UnionValidityChecker<F, N> {
    /// Constructs a new UnionValidityChecker with an empty list of checkers.
    pub fn new() -> Self {
        Self {
            checkers: Vec::new(),
        }
    }

    /// Adds a new validity checker to the union.
    ///
    /// Parameters:
    /// - `checker`: The validity checker to add.
    pub fn add_checker(&mut self, checker: Box<dyn ValidityChecker<F, N>>) {
        self.checkers.push(checker);
    }
}

impl<F: Float, const N: usize> ValidityChecker<F, N> for UnionValidityChecker<F, N> {
    fn is_state_valid(&self, state: &RealVectorState<F, N>) -> bool {
        self.checkers
            .iter()
            .all(|checker| checker.is_state_valid(state))
    }

    fn is_edge_valid(&self, a: &RealVectorState<F, N>, b: &RealVectorState<F, N>) -> bool {
        self.checkers
            .iter()
            .all(|checker| checker.is_edge_valid(a, b))
    }
}
