use crate::obstacles::{SphericalObstacle, StaticSphericalObstacle};
use crate::rrt::RealVectorState;
use num_traits::Float;
use serde::{Deserialize, Serialize};

#[path = "hazard_zone.rs"]
mod hazard_zone;
pub use hazard_zone::HazardZone;

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct SphericalObstacleWithOhz<F: Float, const N: usize> {
    obstacle: StaticSphericalObstacle<F, N>,
    ohz: StaticSphericalObstacle<F, N>,
    /// Optional directed sweep. The original sphere remains the conservative
    /// observation/query bound and preserves the existing wire representation.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    sweep_end: Option<RealVectorState<F, N>>,
    #[serde(default)]
    elliptical_sweep: bool,
    /// Runtime-only normalized affine geometry; the worker validates its wire
    /// representation before constructing it. Legacy serialization is unchanged.
    #[serde(skip)]
    explicit_hazard: Option<HazardZone<F, N>>,
}

impl<F: Float, const N: usize> SphericalObstacleWithOhz<F, N> {
    pub fn new(
        obstacle: StaticSphericalObstacle<F, N>,
        ohz: StaticSphericalObstacle<F, N>,
    ) -> Self {
        Self { obstacle, ohz, sweep_end: None, elliptical_sweep: false, explicit_hazard: None }
    }

    /// Exact occupied disk swept along a verified constant-velocity prediction.
    /// The caller validates the endpoint and observation sphere before calling.
    pub fn with_sweep(mut self, end: RealVectorState<F, N>) -> Self {
        self.explicit_hazard = None;
        self.sweep_end = Some(end);
        self.elliptical_sweep = false;
        self
    }

    /// Conservative ellipse enclosing the same measured sweep as with_sweep.
    pub fn with_ellipse(mut self, end: RealVectorState<F, N>) -> Self {
        self.explicit_hazard = None;
        self.sweep_end = Some(end);
        self.elliptical_sweep = true;
        self
    }

    pub fn with_hazard(mut self, hazard: HazardZone<F, N>) -> Self {
        self.explicit_hazard = Some(hazard);
        self
    }

    /// The one geometry used by native pruning, repair and execution guards.
    pub fn hazard(&self) -> HazardZone<F, N> {
        if let Some(hazard) = &self.explicit_hazard { return hazard.clone(); }
        match self.sweep_end {
            Some(end) if self.elliptical_sweep =>
                HazardZone::ellipse(*self.obstacle.center(), end, self.obstacle.radius()),
            Some(end) => HazardZone::capsule(*self.obstacle.center(), end, self.obstacle.radius()),
            None => HazardZone::Sphere(self.ohz.clone()),
        }
    }

    pub fn obstacle(&self) -> &StaticSphericalObstacle<F, N> {
        &self.obstacle
    }

    pub fn ohz(&self) -> &StaticSphericalObstacle<F, N> {
        &self.ohz
    }
}
