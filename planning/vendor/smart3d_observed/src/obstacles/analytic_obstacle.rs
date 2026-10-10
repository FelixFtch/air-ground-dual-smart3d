use crate::rrt::state::RealVectorState;
use crate::rrt::validity_checker::ValidityChecker;
use num_traits::Float;

/// A trait for analytic obstacles in N-dimensional space.
/// Analytic obstacles have exact functions for containment and intersection checks.
/// They do not require discrete sampling or approximation.
pub trait AnalyticObstacle<F: Float, const N: usize> {
    fn contains(&self, state: &RealVectorState<F, N>) -> bool;
    fn intersects_edge(&self, start: &RealVectorState<F, N>, end: &RealVectorState<F, N>) -> bool;
}

/// A validity checker that uses analytic obstacles to check for collisions.
/// It implements the ValidityChecker trait.
pub struct AnalyticValidityChecker<F: Float, const N: usize, O: AnalyticObstacle<F, N>> {
    obstacles: Vec<O>,
    _phantom: std::marker::PhantomData<F>,
}

impl<F: Float, const N: usize, O: AnalyticObstacle<F, N>> AnalyticValidityChecker<F, N, O> {
    /// Creates a new `AnalyticValidityChecker` with the given obstacles.
    pub fn new(obstacles: Vec<O>) -> Self {
        Self {
            obstacles,
            _phantom: std::marker::PhantomData,
        }
    }
}

impl<F: Float, const N: usize, O: AnalyticObstacle<F, N>> ValidityChecker<F, N>
    for AnalyticValidityChecker<F, N, O>
{
    fn is_state_valid(&self, state: &RealVectorState<F, N>) -> bool {
        self.obstacles
            .iter()
            .all(|obstacle| !obstacle.contains(state))
    }

    fn is_edge_valid(&self, start: &RealVectorState<F, N>, end: &RealVectorState<F, N>) -> bool {
        self.obstacles
            .iter()
            .all(|obstacle| !obstacle.intersects_edge(start, end))
    }
}
