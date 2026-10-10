use crate::obstacles::AnalyticObstacle;
use crate::rrt::state::RealVectorState;
use num_traits::Float;
use serde::{Deserialize, Serialize};

/// A trait for axis-aligned rectangular (hyper‐rectangle) obstacles
pub trait RectangularObstacle<F: Float, const N: usize>: AnalyticObstacle<F, N> {
    /// Returns the minimum corner of the obstacle (smallest coordinates).
    fn min_corner(&self) -> &RealVectorState<F, N>;
    /// Returns the maximum corner of the obstacle (largest coordinates).
    fn max_corner(&self) -> &RealVectorState<F, N>;

    /// Checks if a point lies inside the rectangle.
    fn contains(&self, point: &RealVectorState<F, N>) -> bool {
        // for every dimension, point[i] ∈ [min[i], max[i]]
        (0..N).all(|i| {
            let x = point[i];
            x >= self.min_corner()[i] && x <= self.max_corner()[i]
        })
    }

    /// Checks if a segment [start,end] intersects the rectangle.
    ///
    /// Uses the Liang–Barsky algorithm: for each axis (“slab”) it computes the
    /// entry and exit parameters t₁, t₂ along the parametric line p(t)=start + t*(end-start),
    /// then accumulates a global t_min, t_max. If they overlap within [0,1], there is an intersection.
    fn intersects_edge(&self, start: &RealVectorState<F, N>, end: &RealVectorState<F, N>) -> bool {
        let dir = *end - *start;
        let mut t_min = F::zero();
        let mut t_max = F::one();

        for i in 0..N {
            let s = start[i];
            let d = dir[i];
            let min_i = self.min_corner()[i];
            let max_i = self.max_corner()[i];

            if d == F::zero() {
                // Parallel to slab: if start is outside, no intersection
                if s < min_i || s > max_i {
                    return false;
                }
            } else {
                // Compute intersection t-values with the two planes
                let inv_d = F::one() / d;
                let mut t1 = (min_i - s) * inv_d;
                let mut t2 = (max_i - s) * inv_d;
                // Order them so t1 ≤ t2
                if t1 > t2 {
                    let tmp = t1;
                    t1 = t2;
                    t2 = tmp;
                }
                // Narrow the global interval
                t_min = t_min.max(t1);
                t_max = t_max.min(t2);
                // If empty, no intersection
                if t_min > t_max {
                    return false;
                }
            }
        }

        // Finally, check overlap with segment parameter range [0,1]
        !(t_max < F::zero() || t_min > F::one())
    }

    fn intersects_sphere(&self, sphere_center: &RealVectorState<F, N>, sphere_radius: F) -> bool {
        for i in 0..N {
            if sphere_center[i] < self.min_corner()[i] - sphere_radius
                || sphere_center[i] > self.max_corner()[i] + sphere_radius
            {
                return false;
            }
        }

        true
    }
}

#[derive(Clone, Debug, Serialize, Deserialize)]

pub struct StaticRectangularObstacle<F: Float, const N: usize> {
    min_corner: RealVectorState<F, N>,
    max_corner: RealVectorState<F, N>,
}

impl<F: Float, const N: usize> StaticRectangularObstacle<F, N> {
    /// Creates a new static rectangular obstacle with the given corners.
    pub fn new(min_corner: RealVectorState<F, N>, max_corner: RealVectorState<F, N>) -> Self {
        Self {
            min_corner,
            max_corner,
        }
    }
}

impl<F: Float, const N: usize> RectangularObstacle<F, N> for StaticRectangularObstacle<F, N> {
    fn min_corner(&self) -> &RealVectorState<F, N> {
        &self.min_corner
    }

    fn max_corner(&self) -> &RealVectorState<F, N> {
        &self.max_corner
    }
}

impl<F: Float, const N: usize> AnalyticObstacle<F, N> for StaticRectangularObstacle<F, N> {
    fn contains(&self, state: &RealVectorState<F, N>) -> bool {
        RectangularObstacle::contains(self, state)
    }

    fn intersects_edge(&self, start: &RealVectorState<F, N>, end: &RealVectorState<F, N>) -> bool {
        RectangularObstacle::intersects_edge(self, start, end)
    }
}
