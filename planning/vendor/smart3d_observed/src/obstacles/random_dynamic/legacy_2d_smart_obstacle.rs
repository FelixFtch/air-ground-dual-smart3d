use crate::obstacles::AnalyticObstacle;
use crate::obstacles::{DynamicObstacle, SphericalObstacle, StaticSphericalObstacle};
use crate::rrt::state::RealVectorState;
use num_traits::Float;
use rand::distributions::uniform::SampleUniform;
use rand::Rng;

/// Dynamic obstacle with random movement
/// Random movemement based on legacy Smart2D C++ code:
/// https://github.com/ZongyuanShen/SMART/blob/main/SMART/SMART.cpp
pub struct LegacySmart2DObstacle<F: Float + SampleUniform> {
    sphere: StaticSphericalObstacle<F, 2>,
    speed: F,
    max_leg_distance: F,
    bounds: [(F, F); 2],
    waypoint: RealVectorState<F, 2>,
    reach_threshold: F,
    goal_block_distance: F,
    goal_state: RealVectorState<F, 2>,
}

impl<F: Float + SampleUniform> LegacySmart2DObstacle<F> {
    pub fn new(
        sphere: StaticSphericalObstacle<F, 2>,
        speed: F,
        max_leg_distance: F,
        bounds: [(F, F); 2],
        goal_state: RealVectorState<F, 2>,
    ) -> Self {
        let center = sphere.center().clone();
        Self {
            sphere,
            speed,
            max_leg_distance,
            bounds,
            waypoint: center,
            reach_threshold: F::from(0.5).unwrap(),
            goal_block_distance: F::from(4.0).unwrap(),
            goal_state,
        }
    }

    fn obstacle_inside_area(&self, center: RealVectorState<F, 2>) -> bool {
        let radius = self.sphere.radius();
        for i in 0..2 {
            if center[i] - radius < self.bounds[i].0 || center[i] + radius > self.bounds[i].1 {
                return false;
            }
        }
        true
    }

    fn block_goal_node(&self, state: &RealVectorState<F, 2>) -> bool {
        let threshold = self.goal_block_distance + self.sphere.radius();
        let distance = (state - &self.goal_state).norm();
        distance < threshold
    }
}

impl<F: Float + SampleUniform> SphericalObstacle<F, 2> for LegacySmart2DObstacle<F> {
    fn center(&self) -> &RealVectorState<F, 2> {
        self.sphere.center()
    }

    fn radius(&self) -> F {
        self.sphere.radius()
    }
}

impl<F: Float + SampleUniform> AnalyticObstacle<F, 2> for LegacySmart2DObstacle<F> {
    fn contains(&self, state: &RealVectorState<F, 2>) -> bool {
        SphericalObstacle::contains(&self.sphere, state)
    }

    fn intersects_edge(&self, start: &RealVectorState<F, 2>, end: &RealVectorState<F, 2>) -> bool {
        SphericalObstacle::intersects_edge(&self.sphere, start, end)
    }
}

impl<F: Float + SampleUniform> DynamicObstacle<F> for LegacySmart2DObstacle<F> {
    fn update(&mut self, dt: F) {
        // select new waypoint if current waypoint is reached
        let distance = (&self.waypoint - self.sphere.center()).norm();
        if distance < self.reach_threshold {
            loop {
                // select random distance from 0 to max_leg_distance
                let mut rng = rand::thread_rng();
                let distance = rng.gen_range(F::zero()..self.max_leg_distance);
                // select random angle from 0 to 2 * PI
                let angle = rng.gen_range(F::zero()..F::from(2.0 * std::f64::consts::PI).unwrap());
                // calculate new waypoint
                let dx = distance * angle.cos();
                let dy = distance * angle.sin();
                let displacement = RealVectorState::new([dx, dy]);
                let new_waypoint = self.sphere.center() + &displacement;
                // check if new waypoint is within bounds
                if self.obstacle_inside_area(new_waypoint) && !self.block_goal_node(&new_waypoint) {
                    self.waypoint = new_waypoint;
                    break;
                }
            }
        }

        // move sphere towards waypoint
        let distance = (&self.waypoint - self.sphere.center()).norm();
        if self.speed * dt >= distance {
            self.sphere = StaticSphericalObstacle::new(self.waypoint.clone(), self.sphere.radius());
        } else {
            let heading = F::atan2(
                self.waypoint[1] - self.sphere.center()[1],
                self.waypoint[0] - self.sphere.center()[0],
            );
            let dx = self.speed * dt * heading.cos();
            let dy = self.speed * dt * heading.sin();
            let displacement = RealVectorState::new([dx, dy]);
            self.sphere = StaticSphericalObstacle::new(
                self.sphere.center() + &displacement,
                self.sphere.radius(),
            );
        }
    }
}
