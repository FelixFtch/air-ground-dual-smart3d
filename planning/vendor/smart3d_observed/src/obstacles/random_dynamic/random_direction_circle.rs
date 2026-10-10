use crate::obstacles::{
    AnalyticObstacle, DynamicObstacle, RectangularObstacle, SphericalObstacle,
    StaticRectangularObstacle, StaticSphericalObstacle,
};
use crate::rrt::RealVectorState;
use num_traits::Float;
use rand::distributions::uniform::SampleUniform;
use rand::Rng;

pub struct RandomDirectionCircle<F: Float + SampleUniform> {
    sphere: StaticSphericalObstacle<F, 2>,
    speed: F,
    min_distance: F,
    max_distance: F,
    current_direction: F,
    remaining_distance: F,
    bounds: [(F, F); 2],
    static_rectangles: Vec<StaticRectangularObstacle<F, 2>>,
}

impl<F: Float + SampleUniform> RandomDirectionCircle<F> {
    pub fn new(
        sphere: StaticSphericalObstacle<F, 2>,
        speed: F,
        min_distance: F,
        max_distance: F,
        bounds: [(F, F); 2],
        static_rectangles: Vec<StaticRectangularObstacle<F, 2>>,
    ) -> Self {
        assert!(
            min_distance <= max_distance,
            "min_distance must be <= max_distance"
        );
        let center = sphere.center();
        let (x_min, x_max) = bounds[0];
        let (y_min, y_max) = bounds[1];
        assert!(
            center[0] >= x_min && center[0] <= x_max && center[1] >= y_min && center[1] <= y_max,
            "Initial sphere center must be within bounds"
        );
        for rect in &static_rectangles {
            if rect.intersects_sphere(sphere.center(), sphere.radius()) {
                panic!("Initial sphere intersects with static rectangles");
            }
        }

        Self {
            sphere,
            speed,
            min_distance,
            max_distance,
            current_direction: F::zero(),
            remaining_distance: F::zero(),
            bounds,
            static_rectangles,
        }
    }

    fn is_within_bounds(&self, point: &RealVectorState<F, 2>) -> bool {
        let x = point[0];
        let y = point[1];
        let (x_min, x_max) = self.bounds[0];
        let (y_min, y_max) = self.bounds[1];
        x >= x_min && x <= x_max && y >= y_min && y <= y_max
    }

    fn move_along_leg(&mut self, distance: F) -> bool {
        let dx = distance * self.current_direction.cos();
        let dy = distance * self.current_direction.sin();
        let displacement = RealVectorState::new([dx, dy]);
        let new_center = self.sphere.center() + &displacement;
        debug_assert!(self.is_within_bounds(&new_center), "Movement out of bounds");

        for rect in &self.static_rectangles {
            if rect.intersects_sphere(&new_center, self.sphere.radius()) {
                return false;
            }
        }

        self.sphere = StaticSphericalObstacle::new(new_center, self.sphere.radius());
        true
    }

    fn choose_new_leg(&self) -> (F, F) {
        let mut rng = rand::thread_rng();
        let two_pi = F::from(2.0 * std::f64::consts::PI).expect("Conversion failed");
        loop {
            let angle = rng.gen_range(F::zero()..two_pi);
            let distance = rng.gen_range(self.min_distance..self.max_distance);
            let dx = distance * angle.cos();
            let dy = distance * angle.sin();
            let displacement = RealVectorState::new([dx, dy]);
            let candidate_center = self.sphere.center() + &displacement;
            if self.is_within_bounds(&candidate_center) {
                return (angle, distance);
            }
        }
    }
}

impl<F: Float + SampleUniform> SphericalObstacle<F, 2> for RandomDirectionCircle<F> {
    fn center(&self) -> &RealVectorState<F, 2> {
        self.sphere.center()
    }

    fn radius(&self) -> F {
        self.sphere.radius()
    }
}

impl<F: Float + SampleUniform> AnalyticObstacle<F, 2> for RandomDirectionCircle<F> {
    fn contains(&self, state: &RealVectorState<F, 2>) -> bool {
        SphericalObstacle::contains(&self.sphere, state)
    }

    fn intersects_edge(&self, start: &RealVectorState<F, 2>, end: &RealVectorState<F, 2>) -> bool {
        SphericalObstacle::intersects_edge(&self.sphere, start, end)
    }
}

impl<F: Float + SampleUniform> DynamicObstacle<F> for RandomDirectionCircle<F> {
    fn update(&mut self, dt: F) {
        let mut step = self.speed * dt;

        while step > self.remaining_distance {
            if self.move_along_leg(self.remaining_distance) {
                step = step - self.remaining_distance;
            }
            let (angle, distance) = self.choose_new_leg();
            self.current_direction = angle;
            self.remaining_distance = distance;
        }

        while !self.move_along_leg(step) {
            let (angle, distance) = self.choose_new_leg();
            self.current_direction = angle;
            self.remaining_distance = distance;
        }

        self.remaining_distance = self.remaining_distance - step;
    }
}
