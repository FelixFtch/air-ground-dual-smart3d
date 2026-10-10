use crate::obstacles::AnalyticObstacle;
use crate::rrt::state::RealVectorState;
use num_traits::Float;
use serde::{Deserialize, Serialize};

/// A trait for spherical obstacles
pub trait SphericalObstacle<F: Float, const N: usize>: AnalyticObstacle<F, N> {
    /// Returns the center of the obstacle.
    fn center(&self) -> &RealVectorState<F, N>;
    /// Returns the radius of the obstacle.
    fn radius(&self) -> F;

    /// Checks if a point is inside the sphere.
    fn contains(&self, point: &RealVectorState<F, N>) -> bool {
        let distance_squared = self.center().euclidean_distance_squared(point);
        distance_squared < self.radius().powi(2)
    }

    /// Check if an edge intersects with the sphere.
    fn intersects_edge(&self, start: &RealVectorState<F, N>, end: &RealVectorState<F, N>) -> bool {
        if SphericalObstacle::contains(self, start) || SphericalObstacle::contains(self, end) {
            return true; // One of the endpoints is inside the sphere
        }

        let direction = end - start;
        let center_to_start = start - self.center();
        let a = direction.dot(&direction);
        let b = F::from(2.0).unwrap() * center_to_start.dot(&direction);
        let c = center_to_start.dot(&center_to_start) - self.radius().powi(2);
        let discriminant = b * b - F::from(4.0).unwrap() * a * c;

        if discriminant < F::zero() {
            return false; // No real roots; no intersection
        }

        let sqrt_discriminant = discriminant.sqrt();
        let two_a = F::from(2.0).unwrap() * a;

        let t1 = (-b - sqrt_discriminant) / two_a;
        let t2 = (-b + sqrt_discriminant) / two_a;

        // Check if either intersection point is within the segment [0, 1]
        (t1 >= F::zero() && t1 <= F::one()) || (t2 >= F::zero() && t2 <= F::one())
    }
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct StaticSphericalObstacle<F: Float, const N: usize> {
    center: RealVectorState<F, N>,
    radius: F,
}

impl<F: Float, const N: usize> StaticSphericalObstacle<F, N> {
    pub fn new(center: RealVectorState<F, N>, radius: F) -> Self {
        Self { center, radius }
    }
}

impl<F: Float, const N: usize> SphericalObstacle<F, N> for StaticSphericalObstacle<F, N> {
    fn center(&self) -> &RealVectorState<F, N> {
        &self.center
    }

    fn radius(&self) -> F {
        self.radius
    }
}

impl<F: Float, const N: usize> AnalyticObstacle<F, N> for StaticSphericalObstacle<F, N> {
    fn contains(&self, state: &RealVectorState<F, N>) -> bool {
        SphericalObstacle::contains(self, state)
    }

    fn intersects_edge(&self, start: &RealVectorState<F, N>, end: &RealVectorState<F, N>) -> bool {
        SphericalObstacle::intersects_edge(self, start, end)
    }
}
