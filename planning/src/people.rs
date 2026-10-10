//! Temporary people risk geometry. No person prediction is inserted in OctoMap.
use crate::voxel::{number, string, Identity, Profile};
use json::JsonValue as J;
use smart_3d::{obstacles::StaticSphericalObstacle, rrt::RealVectorState,
    smart::{obstacle::HazardZone, SphericalObstacleWithOhz}};
use std::collections::HashSet;

#[derive(Clone, Debug)]
pub struct PeopleHazard<const N: usize> {
    pub id: String,
    pub center: [f64; 3],
    pub axes: [[f64; 3]; 3],
    pub semi_axes: [f64; 3],
    pub until: f64,
    pub geometry: HazardZone<f32, N>,
    pub wire: J,
}
impl<const N: usize> PeopleHazard<N> {
    pub fn same_geometry(&self, other: &Self) -> bool {
        self.id == other.id && self.center == other.center && self.axes == other.axes
            && self.semi_axes == other.semi_axes
    }
    pub fn obstacle(&self) -> SphericalObstacleWithOhz<f32, N> {
        // An inscribed sphere cannot report a spurious current collision. Exact
        // ellipsoid checks also run through WorldChecker for the whole tree.
        let radius = self.semi_axes[..N].iter().copied().fold(f64::INFINITY, f64::min) as f32;
        SphericalObstacleWithOhz::new(
            StaticSphericalObstacle::new(*self.geometry.center(), radius),
            StaticSphericalObstacle::new(*self.geometry.center(), self.geometry.radius()),
        ).with_hazard(self.geometry.clone())
    }
    pub fn clear_region(&self) -> (RealVectorState<f32, N>, f32) {
        (*self.geometry.center(), self.geometry.radius())
    }
}
fn vec3(v: &J, name: &str) -> Result<[f64; 3], String> {
    if !v.is_array() || v.len() != 3 { return Err(format!("{name}: three coordinates required")); }
    Ok([number(&v[0], name)?, number(&v[1], name)?, number(&v[2], name)?])
}
pub fn parse<const N: usize>(v: &J, now: f64, identity: &Identity, profile: &Profile) -> Result<Vec<PeopleHazard<N>>, String> {
    if !v.is_array() || v.len() > 128 { return Err("people_hazards array required; max128".into()); }
    let mut seen = HashSet::new();
    let mut out = Vec::new();
    for h in v.members() {
        let fail = |detail: &str| format!("people_hazard: {detail}");
        if h["schema"].as_str() != Some("air_ground.people_hazard/v1")
            || h["scene_epoch"].as_str() != Some(&identity.epoch)
            || h["scene_id"].as_str() != Some(&identity.scene_id)
            || h["profile_id"].as_str() != Some(&profile.id) {
            return Err(fail("schema/scene/profile identity mismatch"));
        }
        let legacy_shape = if N == 2 {"ellipse"} else {"ellipsoid"};
        if h["dimension"].as_usize() != Some(N)
            || (h["shape"].as_str() != Some(legacy_shape) && h["shape"].as_str() != Some("box"))
            || h["footprint_included"].as_bool() != Some(true) {
            return Err(fail("dimension/shape/footprint mismatch"));
        }
        let id = string(&h["id"], "people hazard id")?;
        if !seen.insert(id.clone()) { return Err(fail("duplicate person id")); }
        let capture = number(&h["capture_sim_time"], "capture_sim_time")?;
        let reference = number(&h["position_reference_sim_time"], "position_reference_sim_time")?;
        let until = number(&h["valid_until_sim_time"], "valid_until_sim_time")?;
        if capture > reference + 1e-6 || reference > now + 1e-6 || until <= now
            || until <= capture || until - capture > 10. {
            return Err(fail("future, expired or inconsistent times"));
        }
        let horizon = number(&h["horizon_s"], "horizon_s")?;
        if !(0.0..=3.0).contains(&horizon) { return Err(fail("horizon out of bounds")); }
        let velocity = vec3(&h["velocity_xyz"], "velocity_xyz")?;
        let xy = h["velocity_xy_valid"].as_bool().ok_or(fail("XY velocity quality required"))?;
        if h["velocity_z_valid"].as_bool() != Some(false) || velocity[2] != 0.
            || velocity[0].hypot(velocity[1]) > 12. || (!xy && velocity != [0.; 3]) {
            return Err(fail("invalid/unsupported velocity evidence"));
        }
        if !h["source_keys"].is_array() || h["source_keys"].is_empty() || h["source_keys"].len() > 32
            || h["source_keys"].members().any(|k| k.as_str().map_or(true, |s| s.is_empty() || s.len()>1024)) {
            return Err(fail("source evidence required"));
        }
        let body_half = vec3(&h["robot_envelope"]["half_extents_xyz"], "robot_envelope.half_extents_xyz")?;
        let body_offset = vec3(&h["robot_envelope"]["body_offset_m"], "robot_envelope.body_offset_m")?;
        if (0..3).any(|i| (body_half[i]-profile.half[i] as f64).abs()>1e-6
            || (body_offset[i]-profile.offset[i] as f64).abs()>1e-6) {
            return Err(fail("robot envelope mismatch"));
        }
        let center = vec3(&h["center_xyz"], "center_xyz")?;
        let semi_axes = vec3(&h["semi_axes_m"], "semi_axes_m")?;
        if semi_axes[..N].iter().any(|r| *r <= 0. || *r > 100.)
            || (N == 2 && (semi_axes[2] != 0. || (center[2]-profile.planar_z as f64).abs()>1e-6)) {
            return Err(fail("invalid semi axes or planar z"));
        }
        if !h["axes_xyz"].is_array() || h["axes_xyz"].len()!=3 { return Err(fail("three axes required")); }
        let axes = [vec3(&h["axes_xyz"][0], "axis")?, vec3(&h["axes_xyz"][1], "axis")?, vec3(&h["axes_xyz"][2], "axis")?];
        for i in 0..3 { for j in 0..3 {
            let dot = (0..3).map(|k| axes[i][k]*axes[j][k]).sum::<f64>();
            if (dot-if i==j {1.}else{0.}).abs()>1e-6 { return Err(fail("axes must be orthonormal")); }
        }}
        if axes[0][2].abs()>1e-6 || axes[1][2].abs()>1e-6 || axes[2] != [0.,0.,1.] {
            return Err(fail("unknown vertical velocity requires horizontal heading"));
        }
        let speed = velocity[0].hypot(velocity[1]);
        if xy && speed > 1e-8 && ((axes[0][0]-velocity[0]/speed).abs()>1e-6
            || (axes[0][1]-velocity[1]/speed).abs()>1e-6) {
            return Err(fail("heading axis disagrees with valid XY velocity"));
        }
        let geometry = if h["shape"].as_str() == Some("box") {
            HazardZone::oriented_box(std::array::from_fn(|i| center[i]),
                std::array::from_fn(|i| std::array::from_fn(|j| axes[i][j])),
                std::array::from_fn(|i| semi_axes[i]))
        } else {
            HazardZone::ellipsoid(std::array::from_fn(|i| center[i]),
                std::array::from_fn(|i| std::array::from_fn(|j| axes[i][j])),
                std::array::from_fn(|i| semi_axes[i]))
        };
        out.push(PeopleHazard {id, center, axes, semi_axes, until, geometry, wire:h.clone()});
    }
    out.sort_by(|a,b| a.id.cmp(&b.id));
    Ok(out)
}
