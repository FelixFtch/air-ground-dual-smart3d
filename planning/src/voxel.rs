//! Immutable, revisioned OctoMap-derived evidence; absent cells are unknown.
use json::JsonValue as J;
use smart_3d::rrt::{RealVectorState, ValidityChecker};
use std::collections::{HashMap, HashSet};
use std::sync::{
    atomic::{AtomicU64, Ordering},
    Arc,
};

pub type Key = [i32; 3];
#[derive(Clone, Debug, PartialEq)]
pub struct Cell {
    pub occupied: bool,
    pub log_odds: f64,
    /// None preserves restored occupancy without claiming current-epoch evidence.
    pub observed: Option<f64>,
}
#[derive(Clone, Debug, PartialEq)]
pub struct Identity {
    pub scene_id: String,
    pub epoch: String,
    pub content_version: String,
    pub frame_id: String,
    pub resolution: f64,
}
#[derive(Clone, Debug)]
pub struct Map {
    pub identity: Identity,
    pub revision: u64,
    pub watermark: f64,
    pub cells: HashMap<Key, Cell>,
}
#[derive(Default, Debug)]
pub struct Changes {
    pub added: HashSet<Key>,
    pub cleared: Vec<Key>,
    pub unknown_removed: usize,
    pub expired: Vec<Key>,
}

pub fn string(v: &J, name: &str) -> Result<String, String> {
    v.as_str()
        .filter(|x| !x.is_empty() && x.len() <= 256)
        .map(str::to_owned)
        .ok_or(format!("{name}: nonempty string required"))
}
pub fn number(v: &J, name: &str) -> Result<f64, String> {
    v.as_f64()
        .filter(|x| x.is_finite() && x.abs() <= 1e7)
        .ok_or(format!("{name}: finite bounded number required"))
}
pub fn uint(v: &J, name: &str) -> Result<u64, String> {
    v.as_u64()
        .ok_or(format!("{name}: unsigned integer required"))
}
pub fn vector<const N: usize>(v: &J, name: &str) -> Result<[f32; N], String> {
    if !v.is_array() || v.len() != N {
        return Err(format!("{name}: expected {N} coordinates"));
    }
    let mut a = [0.; N];
    for i in 0..N {
        a[i] = number(&v[i], name)? as f32;
    }
    Ok(a)
}
fn identity(v: &J) -> Result<Identity, String> {
    let r = number(&v["resolution"], "resolution")?;
    if !(0.025..=2.).contains(&r) {
        return Err("resolution outside [0.025,2]m".into());
    }
    Ok(Identity {
        scene_id: string(&v["scene_id"], "scene_id")?,
        epoch: string(&v["scene_epoch"], "scene_epoch")?,
        content_version: string(&v["content_version"], "content_version")?,
        frame_id: string(&v["frame_id"], "frame_id")?,
        resolution: r,
    })
}
fn parse_cells(v: &J, delta: bool) -> Result<Vec<(Key, Option<Cell>)>, String> {
    if !v.is_array() || v.len() > 2_000_000 {
        return Err("cell array required; limit 2000000".into());
    }
    let mut seen = HashSet::new();
    let mut out = Vec::with_capacity(v.len());
    for c in v.members() {
        if !c["key"].is_array() || c["key"].len() != 3 {
            return Err("cell.key must be signed [ix,iy,iz]".into());
        }
        let mut k = [0; 3];
        for i in 0..3 {
            k[i] = c["key"][i]
                .as_i32()
                .filter(|n| (-10_000_000..=10_000_000).contains(n))
                .ok_or("cell.key out of range")?;
        }
        if !seen.insert(k) {
            return Err("duplicate cell.key".into());
        }
        let state = match c["state"].as_str() {
            Some("occupied") => Some(true),
            Some("free") => Some(false),
            Some("unknown") if delta => None,
            _ => return Err("invalid cell.state".into()),
        };
        let value = if let Some(occupied) = state {
            Some(Cell {
                occupied,
                log_odds: number(&c["log_odds"], "log_odds")?,
                observed: if c["last_observed_sim_time"].is_null() { None }
                    else { Some(number(&c["last_observed_sim_time"], "last_observed_sim_time")?) },
            })
        } else {
            None
        };
        out.push((k, value));
    }
    Ok(out)
}
impl Map {
    pub fn snapshot(v: &J, old: Option<&Map>) -> Result<(Self, Changes), String> {
        let id = identity(v)?;
        let rev = uint(&v["revision"], "revision")?;
        if let Some(o) = old {
            if id != o.identity {
                return Err("map_identity_changed_reinitialize".into());
            }
            if rev < o.revision {
                return Err("stale_map_revision".into());
            }
        }
        let cells = parse_cells(&v["cells"], false)?
            .into_iter()
            .map(|(k, c)| (k, c.unwrap()))
            .collect();
        let map = Self {
            identity: id,
            revision: rev,
            watermark: number(
                &v["capture_sim_time_watermark"],
                "capture_sim_time_watermark",
            )?,
            cells,
        };
        if let Some(o) = old {
            if rev == o.revision && (map.cells != o.cells || map.watermark != o.watermark) {
                return Err("map_content_changed_without_revision".into());
            }
        }
        let ch = map.diff(old);
        Ok((map, ch))
    }
    pub fn delta(v: &J, old: &Map) -> Result<(Self, Changes), String> {
        if identity(v)? != old.identity {
            return Err("map_identity_changed_reinitialize".into());
        }
        if uint(&v["base_revision"], "base_revision")? != old.revision {
            return Err("snapshot_required: map_revision_gap".into());
        }
        let rev = uint(&v["revision"], "revision")?;
        if rev < old.revision {
            return Err("stale_map_revision".into());
        }
        let changes = parse_cells(&v["changed_cells"], true)?;
        if rev == old.revision && !changes.is_empty() {
            return Err("map_content_changed_without_revision".into());
        }
        let mut next = old.clone();
        next.revision = rev;
        next.watermark = number(
            &v["capture_sim_time_watermark"],
            "capture_sim_time_watermark",
        )?;
        for (k, c) in &changes {
            if let Some(c) = c {
                next.cells.insert(*k, c.clone());
            } else {
                next.cells.remove(k);
            }
        }
        if rev == old.revision && next.watermark != old.watermark {
            return Err("map_content_changed_without_revision".into());
        }
        // Compact mapper deltas preserve occupancy classes but refresh evidence time.
        if !v["refreshes"].is_null() {
            if !v["refreshes"].is_array() {
                return Err("refreshes must be an array".into());
            }
            for group in v["refreshes"].members() {
                let time = number(&group["capture_sim_time"], "refresh capture_sim_time")?;
                if !group["keys"].is_array() || group["keys"].len() > 2_000_000 {
                    return Err("refresh keys array required".into());
                }
                for value in group["keys"].members() {
                    if !value.is_array() || value.len() != 3 {
                        return Err("refresh key must have 3 integers".into());
                    }
                    let mut k = [0; 3];
                    for i in 0..3 {
                        k[i] = value[i]
                            .as_i32()
                            .filter(|n| (-10_000_000..=10_000_000).contains(n))
                            .ok_or("refresh key invalid")?;
                    }
                    let cell = next
                        .cells
                        .get_mut(&k)
                        .ok_or("snapshot_required: refresh references unknown cell")?;
                    if cell.observed.map_or(false, |observed| time < observed) {
                        return Err("stale_cell_refresh".into());
                    }
                    cell.observed = Some(time);
                }
            }
        }
        if rev == old.revision && next.cells != old.cells {
            return Err("map_content_changed_without_revision".into());
        }
        // Only changed keys can change occupancy. Refresh groups above update
        // evidence timestamps, never geometry. Classify only after every input
        // has validated, preserving the immutable clone/commit transaction.
        let ch = next.diff_changed(old, changes.iter().map(|(k, _)| k));
        Ok((next, ch))
    }
    fn diff_changed<'a>(&self, old: &Map, keys: impl Iterator<Item = &'a Key>) -> Changes {
        let mut c = Changes::default();
        for k in keys {
            let was_occupied = old.cells.get(k).map_or(false, |v| v.occupied);
            match self.cells.get(k) {
                Some(v) if v.occupied && !was_occupied => {
                    c.added.insert(*k);
                }
                Some(v) if !v.occupied && was_occupied => c.cleared.push(*k),
                None if was_occupied => c.expired.push(*k),
                _ => {}
            }
        }
        c.cleared.sort();
        c.expired.sort();
        c.unknown_removed = c.expired.len();
        c
    }
    fn diff(&self, old: Option<&Map>) -> Changes {
        let mut c = Changes::default();
        for (k, v) in &self.cells {
            let was = old
                .and_then(|m| m.cells.get(k))
                .map_or(false, |v| v.occupied);
            if v.occupied && !was {
                c.added.insert(*k);
            } else if !v.occupied && was {
                c.cleared.push(*k);
            }
        }
        if let Some(o) = old {
            c.expired = o
                .cells
                .iter()
                .filter(|(k, v)| v.occupied && !self.cells.contains_key(*k))
                .map(|(k, _)| *k)
                .collect();
            c.expired.sort();
            c.unknown_removed = c.expired.len();
        }
        c.cleared.sort();
        c
    }
    pub fn json_identity(&self) -> J {
        json::object! {scene_id:self.identity.scene_id.clone(),scene_epoch:self.identity.epoch.clone(),content_version:self.identity.content_version.clone(),frame_id:self.identity.frame_id.clone(),resolution:self.identity.resolution,revision:self.revision,capture_sim_time_watermark:self.watermark}
    }
}
#[derive(Clone, Debug)]
pub struct Profile {
    pub id: String,
    pub half: [f32; 3],
    pub offset: [f32; 3],
    pub planar_z: f32,
    /// Extra 3D clearance around a connected occupied group. A lone voxel stays
    /// at the body size so one speck does not become a large obstacle.
    pub cluster_inflation_m: f32,
}
impl Profile {
    pub fn parse(v: &J) -> Result<Self, String> {
        let half = vector(&v["body_half_extents_m"], "body_half_extents_m")?;
        if half.iter().any(|&x| x <= 0. || x > 5.) {
            return Err("body half extents must be in (0,5]m".into());
        }
        let cluster_inflation_m = if v["cluster_inflation_m"].is_null() {
            0.
        } else {
            let value = number(&v["cluster_inflation_m"], "cluster_inflation_m")? as f32;
            if !(0. ..=2.).contains(&value) {
                return Err("cluster_inflation_m must be in [0,2]m".into());
            }
            value
        };
        Ok(Self {
            id: string(&v["profile_id"], "profile_id")?,
            half,
            offset: vector(&v["body_offset_m"], "body_offset_m")?,
            planar_z: number(&v["planar_z"], "planar_z")? as f32,
            cluster_inflation_m,
        })
    }
    pub fn xyz<const N: usize>(&self, p: &[f32; N]) -> [f32; 3] {
        [p[0], p[1], if N == 3 { p[2] } else { self.planar_z }]
    }
    pub fn clear_center<const N: usize>(&self, k: Key, res: f64) -> RealVectorState<f32, N> {
        RealVectorState::new(std::array::from_fn(|i| {
            ((k[i] as f64 + 0.5) * res) as f32 - self.offset[i]
        }))
    }
}
#[derive(Default)]
pub struct Counters {
    pub states: AtomicU64,
    pub edges: AtomicU64,
    pub cell_tests: AtomicU64,
    pub oversize_sweeps: AtomicU64,
}
impl Counters {
    pub fn json(&self) -> J {
        json::object! {state_checks:self.states.load(Ordering::Relaxed),edge_checks:self.edges.load(Ordering::Relaxed),voxel_tests:self.cell_tests.load(Ordering::Relaxed),oversize_sweeps:self.oversize_sweeps.load(Ordering::Relaxed)}
    }
}
#[derive(Clone)]
pub struct Checker<const N: usize> {
    pub map: Arc<Map>,
    pub profile: Profile,
    pub ranges: [(f32, f32); N],
    pub added_only: Option<Arc<HashSet<Key>>>,
    pub counters: Arc<Counters>,
}
impl<const N: usize> Checker<N> {
    pub fn xyz(&self, p: &RealVectorState<f32, N>) -> [f32; 3] {
        self.profile.xyz(p.values())
    }
    fn bounds(&self, p: &RealVectorState<f32, N>) -> bool {
        (0..N).all(|i| {
            p.values()[i].is_finite()
                && p.values()[i] >= self.ranges[i].0
                && p.values()[i] <= self.ranges[i].1
        })
    }
    pub fn sweep(&self, a: [f32; 3], b: [f32; 3], require_free: bool) -> bool {
        let mut lo = [0i32; 3];
        let mut hi = [0i32; 3];
        let r = self.map.identity.resolution;
        let ca: [f64; 3] = std::array::from_fn(|i| a[i] as f64 + self.profile.offset[i] as f64);
        let cb: [f64; 3] = std::array::from_fn(|i| b[i] as f64 + self.profile.offset[i] as f64);
        let cluster = self.profile.cluster_inflation_m.max(0.) as f64;
        for i in 0..3 {
            let h = self.profile.half[i] as f64 + cluster;
            lo[i] = ((ca[i].min(cb[i]) - h - 1e-7) / r).floor() as i32;
            hi[i] = ((ca[i].max(cb[i]) + h + 1e-7) / r).floor() as i32;
        }
        let volume = (0..3).try_fold(1u64, |n, i| {
            n.checked_mul((hi[i] as i64 - lo[i] as i64 + 1) as u64)
        });
        if volume.map_or(true, |v| v > 2_000_000) {
            self.counters
                .oversize_sweeps
                .fetch_add(1, Ordering::Relaxed);
            return false;
        }
        for x in lo[0]..=hi[0] {
            for y in lo[1]..=hi[1] {
                for z in lo[2]..=hi[2] {
                    let k = [x, y, z];
                    let blocked = if let Some(added) = &self.added_only {
                        added.contains(&k)
                    } else {
                        self.map.cells.get(&k).map_or(require_free, |c|
                            c.occupied || (require_free && c.observed.is_none()))
                    };
                    if !blocked {
                        continue;
                    }
                    self.counters.cell_tests.fetch_add(1, Ordering::Relaxed);
                    let extra = if cluster > 0. && self.occupied_neighbor(k) {
                        cluster
                    } else {
                        0.
                    };
                    let min = std::array::from_fn(|i| {
                        k[i] as f64 * r - self.profile.half[i] as f64 - extra
                    });
                    let max = std::array::from_fn(|i| {
                        (k[i] as f64 + 1.) * r + self.profile.half[i] as f64 + extra
                    });
                    if segment_box(ca, cb, min, max) {
                        return false;
                    }
                }
            }
        }
        true
    }
    /// A voxel belongs to a group when another occupied voxel touches it.
    /// Isolated returns keep body clearance only.
    fn occupied_neighbor(&self, k: Key) -> bool {
        const STEPS: [[i32; 3]; 6] = [
            [1, 0, 0], [-1, 0, 0], [0, 1, 0], [0, -1, 0], [0, 0, 1], [0, 0, -1],
        ];
        STEPS.iter().any(|step| {
            let next = [k[0] + step[0], k[1] + step[1], k[2] + step[2]];
            self.map.cells.get(&next).map(|cell| cell.occupied).unwrap_or(false)
        })
    }
}
/// Exact closed segment vs expanded voxel AABB (Minkowski swept body test).
pub fn segment_box(a: [f64; 3], b: [f64; 3], lo: [f64; 3], hi: [f64; 3]) -> bool {
    let (mut t0, mut t1) = (0f64, 1f64);
    for i in 0..3 {
        let d = b[i] - a[i];
        if d.abs() < 1e-12 {
            if a[i] < lo[i] - 1e-8 || a[i] > hi[i] + 1e-8 {
                return false;
            }
        } else {
            let (mut u, mut v) = ((lo[i] - a[i]) / d, (hi[i] - a[i]) / d);
            if u > v {
                std::mem::swap(&mut u, &mut v)
            }
            t0 = t0.max(u);
            t1 = t1.min(v);
            if t0 > t1 + 1e-8 {
                return false;
            }
        }
    }
    true
}
impl<const N: usize> ValidityChecker<f32, N> for Checker<N> {
    fn is_state_valid(&self, p: &RealVectorState<f32, N>) -> bool {
        self.counters.states.fetch_add(1, Ordering::Relaxed);
        self.bounds(p) && self.sweep(self.xyz(p), self.xyz(p), false)
    }
    fn is_edge_valid(&self, a: &RealVectorState<f32, N>, b: &RealVectorState<f32, N>) -> bool {
        self.counters.edges.fetch_add(1, Ordering::Relaxed);
        self.bounds(a) && self.bounds(b) && self.sweep(self.xyz(a), self.xyz(b), false)
    }
}
