use crate::obstacles::{AnalyticObstacle, DynamicObstacle, SphericalObstacle};
use crate::obstacles::{StaticRectangularObstacle, StaticSphericalObstacle};
use crate::rrt::sampling::SamplingDistribution;
use crate::rrt::RealVectorState;
use num_traits::Float;

pub struct RandomWaypointSphere<F: Float, const N: usize> {
    sphere: StaticSphericalObstacle<F, N>,
    speed: F,
    sampler: Box<dyn SamplingDistribution<F, N>>,
    restricted_regions: Vec<StaticRectangularObstacle<F, N>>,
    target: RealVectorState<F, N>,
}

impl<F: Float, const N: usize> RandomWaypointSphere<F, N> {
    pub fn new(
        sphere: StaticSphericalObstacle<F, N>,
        speed: F,
        mut sampler: Box<dyn SamplingDistribution<F, N>>,
        restricted_regions: Vec<StaticRectangularObstacle<F, N>>,
    ) -> Self {
        let target = sampler.sample();
        Self {
            sphere,
            speed,
            sampler,
            restricted_regions,
            target,
        }
    }

    fn sample_new_target(&mut self) {
        loop {
            self.target = self.sampler.sample();
            if !self.in_restricted_region(&self.target) {
                break;
            }
        }
    }

    fn in_restricted_region(&self, state: &RealVectorState<F, N>) -> bool {
        for region in &self.restricted_regions {
            if region.contains(state) {
                return true;
            }
        }
        false
    }
}

impl<F: Float, const N: usize> SphericalObstacle<F, N> for RandomWaypointSphere<F, N> {
    fn center(&self) -> &RealVectorState<F, N> {
        self.sphere.center()
    }

    fn radius(&self) -> F {
        self.sphere.radius()
    }
}

impl<F: Float, const N: usize> AnalyticObstacle<F, N> for RandomWaypointSphere<F, N> {
    fn contains(&self, state: &RealVectorState<F, N>) -> bool {
        SphericalObstacle::contains(&self.sphere, state)
    }

    fn intersects_edge(&self, start: &RealVectorState<F, N>, end: &RealVectorState<F, N>) -> bool {
        SphericalObstacle::intersects_edge(&self.sphere, start, end)
    }
}

impl<F: Float, const N: usize> DynamicObstacle<F> for RandomWaypointSphere<F, N> {
    fn update(&mut self, dt: F) {
        let direction = &self.target - self.sphere.center();
        let distance = direction.norm();
        let step = self.speed * dt;
        if distance < step || self.in_restricted_region(&self.target) {
            self.sample_new_target();
        }
        let step = step.min(distance);
        let step_vector = (direction / distance) * step;
        self.sphere =
            StaticSphericalObstacle::new(self.sphere.center() + &step_vector, self.sphere.radius());
    }
}
