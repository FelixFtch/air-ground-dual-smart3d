pub mod voxel;
pub mod people;
use people::PeopleHazard;
use json::JsonValue as J;
use smart_3d::{
    rrt::{
        termination::{MaxIterationsTermination, MaxTimeTermination, OrTermination},
        KdTreeNearestNeighbors, RRTstar, RealVectorState, SamplingDistribution, ValidityChecker,
    },
    smart::SMART,
};
use std::{
    collections::HashSet,
    sync::Arc,
    time::{Duration, Instant},
};
use voxel::*;
const LOCAL_REACTION_RADIUS_M: f32 = 4.0;
type State<const N: usize> = RealVectorState<f32, N>;
type Core<const N: usize> = SMART<f32, N, KdTreeNearestNeighbors<f32, N>>;

/// Low-discrepancy sampling feeds the unchanged RRT*/SMART insertion rules.
/// It makes contract tests and offline replays repeatable without a second planner.
struct Halton<const N: usize> {
    ranges: [(f32, f32); N],
    index: u64,
}
impl<const N: usize> SamplingDistribution<f32, N> for Halton<N> {
    fn sample(&mut self) -> State<N> {
        self.index += 1;
        let primes = [2, 3, 5];
        State::new(std::array::from_fn(|i| {
            let (mut n, mut f, mut v) = (self.index, 1f64, 0f64);
            while n > 0 {
                f /= primes[i] as f64;
                v += f * (n % primes[i]) as f64;
                n /= primes[i];
            }
            self.ranges[i].0 + (self.ranges[i].1 - self.ranges[i].0) * v as f32
        }))
    }
}
#[derive(Clone, Debug, PartialEq)]
pub struct DynamicBox {
    pub id: String,
    pub center: [f32; 3],
    pub half: [f32; 3],
    pub until: f64,
}
impl DynamicBox {
    /// Lease metadata is independent of the occupied geometry. Compare against
    /// the previous installed layer (including newly expired boxes) so removal
    /// still queues recovery, but renewal does not manufacture a clear event.
    fn same_geometry(&self, other: &Self) -> bool {
        self.id == other.id && self.center == other.center && self.half == other.half
    }
}
fn boxes(v: &J, now: f64) -> Result<Vec<DynamicBox>, String> {
    if !v.is_array() || v.len() > 128 {
        return Err("dynamic_boxes array required; max128".into());
    }
    let mut seen = HashSet::new();
    let mut out = Vec::new();
    for b in v.members() {
        let id = string(&b["id"], "dynamic box id")?;
        if !seen.insert(id.clone()) {
            return Err("duplicate dynamic box id".into());
        }
        let half = vector::<3>(&b["half_extents_xyz"], "half_extents_xyz")?;
        if half.iter().any(|x| *x <= 0. || *x > 10.) {
            return Err("dynamic half extents outside (0,10]".into());
        }
        let until = number(&b["valid_until_sim_time"], "valid_until_sim_time")?;
        let center = vector(&b["center_xyz"], "center_xyz")?;
        if until > now {
            out.push(DynamicBox {
                id,
                center,
                half,
                until,
            });
        }
    }
    out.sort_by(|a, b| a.id.cmp(&b.id));
    Ok(out)
}
#[derive(Clone)]
struct WorldChecker<const N: usize> {
    vox: Checker<N>,
    dynamic: Arc<Vec<DynamicBox>>,
    people: Arc<Vec<PeopleHazard<N>>>,
}
impl<const N: usize> WorldChecker<N> {
    fn dynamic_clear(&self, a: &State<N>, b: &State<N>) -> bool {
        let ca = self.vox.xyz(a);
        let cb = self.vox.xyz(b);
        let ca = std::array::from_fn(|i| ca[i] as f64 + self.vox.profile.offset[i] as f64);
        let cb = std::array::from_fn(|i| cb[i] as f64 + self.vox.profile.offset[i] as f64);
        self.dynamic.iter().all(|d| {
            !segment_box(
                ca,
                cb,
                std::array::from_fn(|i| {
                    (d.center[i] - d.half[i] - self.vox.profile.half[i]) as f64
                }),
                std::array::from_fn(|i| {
                    (d.center[i] + d.half[i] + self.vox.profile.half[i]) as f64
                }),
            )
        })
    }
}
impl<const N: usize> ValidityChecker<f32, N> for WorldChecker<N> {
    fn is_state_valid(&self, p: &State<N>) -> bool {
        self.vox.is_state_valid(p) && self.dynamic_clear(p, p)
            && self.people.iter().all(|h| !h.geometry.contains(p))
    }
    fn is_edge_valid(&self, a: &State<N>, b: &State<N>) -> bool {
        self.vox.is_edge_valid(a, b) && self.dynamic_clear(a, b)
            && self.people.iter().all(|h| !h.geometry.intersects_edge(a,b))
    }
}
fn budget(iterations: usize, millis: u64) -> OrTermination {
    OrTermination::new(vec![
        Box::new(MaxIterationsTermination::new(iterations)),
        Box::new(MaxTimeTermination::new(Duration::from_millis(millis))),
    ])
}
fn limit(v: &J, name: &str, default: usize, max: usize) -> Result<usize, String> {
    if v.is_null() {
        return Ok(default);
    }
    let n = uint(v, name)? as usize;
    if n > max {
        return Err(format!("{name} exceeds {max}"));
    }
    Ok(n)
}
fn jarray<const N: usize>(a: [f32; N]) -> J {
    J::Array(a.into_iter().map(J::from).collect())
}
fn json_state<const N: usize>(p: &State<N>, profile: &Profile) -> J {
    jarray(profile.xyz(p.values()))
}

pub struct Session<const N: usize> {
    robot: String,
    session: String,
    instance: u64,
    goal_revision: u64,
    goal: State<N>,
    current: State<N>,
    ranges: [(f32, f32); N],
    profile: Profile,
    map: Arc<Map>,
    dynamic: Arc<Vec<DynamicBox>>,
    dynamic_revision: u64,
    people: Arc<Vec<PeopleHazard<N>>>,
    people_revision: u64,
    sim_time: f64,
    core: Core<N>,
    counters: Arc<Counters>,
    repair_iterations: usize,
    repair_ms: u64,
    path_seq: u64,
}
impl<const N: usize> Session<N> {
    fn checker(
        &self,
        map: Arc<Map>,
        dynamic: Arc<Vec<DynamicBox>>,
        people: Arc<Vec<PeopleHazard<N>>>,
        added: Option<Arc<HashSet<Key>>>,
    ) -> WorldChecker<N> {
        WorldChecker {
            vox: Checker {
                map,
                profile: self.profile.clone(),
                ranges: self.ranges,
                added_only: added,
                counters: self.counters.clone(),
            },
            dynamic,
            people,
        }
    }
    fn init(v: &J, instance: u64) -> Result<Self, String> {
        if v["protocol"].as_str() != Some("air_ground_smart3d.v1") {
            return Err("protocol must be air_ground_smart3d.v1".into());
        }
        let robot = string(&v["robot_id"], "robot_id")?;
        let session = string(&v["session_id"], "session_id")?;
        let goal_revision = uint(&v["goal_revision"], "goal_revision")?;
        let start = State::new(vector(&v["start"], "start")?);
        let goal = State::new(vector(&v["goal"], "goal")?);
        let profile = Profile::parse(&v["profile"])?;
        if !v["ranges"].is_array() || v["ranges"].len() != N {
            return Err("ranges must match dimension".into());
        }
        let mut ranges = [(0., 0.); N];
        for i in 0..N {
            let a = vector::<2>(&v["ranges"][i], "range")?;
            if a[1] <= a[0] || a[1] - a[0] > 1000. {
                return Err("range width must be (0,1000]m".into());
            }
            ranges[i] = (a[0], a[1]);
        }
        let (map, _) = Map::snapshot(&v["map_snapshot"], None)?;
        let sim_time = if v["sim_time"].is_null() {
            map.watermark
        } else {
            number(&v["sim_time"], "sim_time")?
        };
        let dynamic = if v["dynamic_boxes"].is_null() {
            Vec::new()
        } else {
            boxes(&v["dynamic_boxes"], sim_time)?
        };
        let people = if v["people_hazards"].is_null() { Vec::new() }
            else { people::parse(&v["people_hazards"], sim_time, &map.identity, &profile)? };
        let dynamic_revision = if v["dynamic_revision"].is_null() {
            0
        } else {
            uint(&v["dynamic_revision"], "dynamic_revision")?
        };
        let iterations = limit(&v["initial_iterations"], "initial_iterations", 1200, 20000)?;
        let repair_iterations = limit(&v["repair_iterations"], "repair_iterations", 128, 5000)?;
        let initial_ms = limit(&v["initial_budget_ms"], "initial_budget_ms", 1500, 10000)? as u64;
        let repair_ms = limit(&v["repair_budget_ms"], "repair_budget_ms", 50, 1000)? as u64;
        let counters = Arc::new(Counters::default());
        let map = Arc::new(map);
        let dynamic = Arc::new(dynamic);
        let people = Arc::new(people);
        let checker = WorldChecker {
            vox: Checker {
                map: map.clone(),
                profile: profile.clone(),
                ranges,
                added_only: None,
                counters: counters.clone(),
            },
            dynamic: dynamic.clone(),
            people: people.clone(),
        };
        if !checker.is_state_valid(&goal) {
            return Err("goal_blocked_or_out_of_bounds".into());
        }
        if !checker.is_state_valid(&start) {
            return Err("start_blocked_or_out_of_bounds".into());
        }
        let volume = ranges.iter().map(|(a, b)| b - a).product();
        let gamma = smart_3d::rrt::rrt_star::optimal_gamma(volume, N);
        let mut rrt: RRTstar<f32, N, KdTreeNearestNeighbors<f32, N>> = RRTstar::new(
            goal,
            start,
            0.3,
            Box::new(checker),
            Box::new(Halton { ranges, index: 0 }),
            1.,
            1.8,
            gamma,
        );
        rrt.plan_until(&mut budget(iterations, initial_ms));
        let (nodes, nn, checker, sampler) = rrt.deconstruct_into_components();
        let mut core = SMART::new_from_initial_rrt_star_tree(
            &nodes, nn, start, goal, 0.3, checker, sampler, LOCAL_REACTION_RADIUS_M, 1.5, 1.5, 8., 1.8, 1.8, false, 0.01,
        );
        core.initial_plan_add_node_at_start();
        core.initial_plan_build_path();
        Ok(Self {
            robot,
            session,
            instance,
            goal_revision,
            goal,
            current: start,
            ranges,
            profile,
            map,
            dynamic,
            dynamic_revision,
            people,
            people_revision: 0,
            sim_time,
            core,
            counters,
            repair_iterations,
            repair_ms,
            path_seq: 0,
        })
    }
    fn call(&mut self, v: &J) -> Result<J, String> {
        let started = Instant::now();
        if v["robot_id"].as_str() != Some(&self.robot)
            || v["session_id"].as_str() != Some(&self.session)
        {
            return Err("robot_or_session_mismatch".into());
        }
        let op = string(&v["op"], "op")?;
        if !["plan", "update", "state", "set_goal"].contains(&op.as_str()) {
            return Err("unknown op".into());
        }
        let requested_rev = uint(&v["goal_revision"], "goal_revision")?;
        if op != "set_goal" && requested_rev != self.goal_revision {
            return Err("goal_revision_mismatch".into());
        }
        if op == "state" {
            return Ok(self.output(v, json::object! {operation:"state"}, started));
        }
        let current = State::new(vector(&v["current"], "current")?);
        let goal = if op == "set_goal" {
            State::new(vector(&v["goal"], "goal")?)
        } else {
            self.goal
        };
        let changed_goal = goal != self.goal;
        if op == "set_goal"
            && (requested_rev < self.goal_revision
                || (changed_goal && requested_rev == self.goal_revision))
        {
            return Err("new_goal_requires_newer_revision".into());
        }
        if !v["map_snapshot"].is_null() && !v["map_delta"].is_null() {
            return Err("use map_snapshot OR map_delta".into());
        }
        let (map, changes) = if !v["map_snapshot"].is_null() {
            let (m, c) = Map::snapshot(&v["map_snapshot"], Some(&self.map))?;
            (Arc::new(m), c)
        } else if !v["map_delta"].is_null() {
            let (m, c) = Map::delta(&v["map_delta"], &self.map)?;
            (Arc::new(m), c)
        } else {
            (self.map.clone(), Changes::default())
        };
        let now = if v["sim_time"].is_null() {
            self.sim_time.max(map.watermark)
        } else {
            number(&v["sim_time"], "sim_time")?
        };
        if now < self.sim_time {
            return Err("stale_sim_time".into());
        }
        let dyn_rev = if v["dynamic_revision"].is_null() {
            self.dynamic_revision
        } else {
            uint(&v["dynamic_revision"], "dynamic_revision")?
        };
        if dyn_rev < self.dynamic_revision {
            return Err("stale_dynamic_revision".into());
        }
        let dyns = if v["dynamic_boxes"].is_null() {
            self.dynamic
                .iter()
                .filter(|d| d.until > now)
                .cloned()
                .collect()
        } else {
            boxes(&v["dynamic_boxes"], now)?
        };
        let old_active: Vec<_> = self
            .dynamic
            .iter()
            .filter(|d| d.until > now)
            .cloned()
            .collect();
        if !v["dynamic_boxes"].is_null() && dyns != old_active && dyn_rev == self.dynamic_revision {
            return Err("dynamic_content_changed_without_revision".into());
        }
        let dynamic = Arc::new(dyns);
        let people = Arc::new(if v["people_hazards"].is_null() {
            self.people.iter().filter(|h| h.until > now).cloned().collect()
        } else { people::parse(&v["people_hazards"], now, &map.identity, &self.profile)? });
        let full = self.checker(map.clone(), dynamic.clone(), people.clone(), None);
        if op == "set_goal" && !full.is_state_valid(&goal) {
            return Err("goal_blocked_or_out_of_bounds".into());
        }
        if changed_goal && self.core.get_nodes().len() >= 250000 {
            return Err("tree_capacity".into());
        }
        // All validation above precedes mutation. Full-tree obstacle pruning is not
        // covered by the native repair budget; total wall time is reported separately.
        let nodes_before = self.core.get_nodes().len();
        let edges_before = self
            .core
            .get_nodes()
            .iter()
            .filter(|n| n.parent.is_some())
            .count();
        let dynamic_metadata_changed = *dynamic != *self.dynamic;
        let dynamic_added_or_moved_boxes = dynamic
            .iter()
            .filter(|d| !self.dynamic.iter().any(|old| d.same_geometry(old)))
            .count();
        let dynamic_removed_or_moved_boxes = self
            .dynamic
            .iter()
            .filter(|old| !dynamic.iter().any(|d| old.same_geometry(d)))
            .count();
        let dynamic_changed =
            dynamic_added_or_moved_boxes > 0 || dynamic_removed_or_moved_boxes > 0;
        let dynamic_ttl_only_refresh = dynamic_metadata_changed && !dynamic_changed;
        let people_metadata_changed = people.len() != self.people.len()
            || people.iter().zip(self.people.iter()).any(|(a,b)| a.wire != b.wire);
        let people_added_or_moved = people.iter()
            .filter(|h| !self.people.iter().any(|old| h.same_geometry(old))).count();
        let people_removed_or_moved = self.people.iter()
            .filter(|old| !people.iter().any(|h| old.same_geometry(h))).count();
        let people_changed = people_added_or_moved > 0 || people_removed_or_moved > 0;
        let people_metadata_only_refresh = people_metadata_changed && !people_changed;
        let old_people = self.people.clone();
        let prune_scan_triggered = !changes.added.is_empty() || dynamic_changed || people_changed;
        let old_dynamic = self.dynamic.clone();
        self.map = map;
        self.dynamic = dynamic;
        self.dynamic_revision = dyn_rev;
        self.people = people;
        if people_changed { self.people_revision += 1; }
        self.sim_time = now;
        self.current = current;
        let mut native = "CheckerRefreshed".to_string();
        if prune_scan_triggered {
            let added = self.checker(
                self.map.clone(),
                self.dynamic.clone(),
                self.people.clone(),
                Some(Arc::new(changes.added.clone())),
            );
            native = format!(
                "{:?}",
                self.core.update_new_static_obstacles_with_cursor(
                    Box::new(full.clone()),
                    &added,
                    Some(&mut budget(self.repair_iterations, self.repair_ms)),
                    current,
                    0
                )
            );
        } else {
            self.core
                .set_static_validity_checker(Box::new(full.clone()));
        }
        let mut regions: Vec<_> = changes
            .cleared
            .iter()
            .chain(changes.expired.iter())
            .map(|&k| {
                (
                    self.profile.clear_center(k, self.map.identity.resolution),
                    self.map.identity.resolution as f32 * 0.8661
                        + self.profile.half.iter().map(|v| v * v).sum::<f32>().sqrt(),
                )
            })
            .collect();
        if dynamic_changed {
            for d in old_dynamic.iter() {
                if !self.dynamic.iter().any(|next| d.same_geometry(next)) {
                    regions.push((
                        State::new(std::array::from_fn(|i| {
                            d.center[i] - self.profile.offset[i]
                        })),
                        d.half.iter().map(|v| v * v).sum::<f32>().sqrt()
                            + self.profile.half.iter().map(|v| v * v).sum::<f32>().sqrt(),
                    ));
                }
            }
        }
        let map_clear_regions_queued = changes.cleared.len() + changes.expired.len();
        let dynamic_clear_regions_queued = regions.len() - map_clear_regions_queued;
        if people_changed {
            for old in old_people.iter() {
                if !self.people.iter().any(|h| old.same_geometry(h)) {
                    regions.push(old.clear_region());
                }
            }
        }
        let people_clear_regions_queued = regions.len() - map_clear_regions_queued - dynamic_clear_regions_queued;
        self.core.queue_static_clear_regions(&regions);
        let clear = self
            .core
            .maintain_static_clear_regions(64, 512, 64, Duration::from_millis(10));
        let coverage =
            self.core
                .maintain_static_clear_coverage(64, 8, 250000, Duration::from_millis(10));
        let mut reroot = J::Null;
        if changed_goal {
            let when = Instant::now();
            let (old_root_new_id, retained_nodes, retained_edges, connected) = self
                .core
                .reroot_goal(goal, current)
                .map_err(str::to_owned)?;
            reroot = json::object! {old_root_id:0,old_root_new_id:old_root_new_id,retained_nodes:retained_nodes,retained_edges:retained_edges,goal_connected:connected,reroot_ms:when.elapsed().as_secs_f64()*1000.};
        }
        if op == "set_goal" {
            self.goal = goal;
            self.goal_revision = requested_rev;
        }
        self.core.refresh_current_path(current);
        let native_people = self.people.iter().map(|h| h.obstacle()).collect();
        let refresh = format!(
            "{:?}",
            self.core.update(
                current,
                0,
                &native_people,
                Some(&mut budget(self.repair_iterations, self.repair_ms))
            )
        );
        let mut metrics = json::object! {operation:op,native_map_result:native,native_plan_result:refresh,nodes_before:nodes_before,edges_before:edges_before,occupied_added:changes.added.len(),occupied_cleared:changes.cleared.len(),occupied_expired_to_unknown:changes.unknown_removed,dynamic_changed:dynamic_changed,dynamic_metadata_changed:dynamic_metadata_changed,dynamic_ttl_only_refresh:dynamic_ttl_only_refresh,dynamic_added_or_moved_boxes:dynamic_added_or_moved_boxes,dynamic_removed_or_moved_boxes:dynamic_removed_or_moved_boxes,prune_scan_triggered:prune_scan_triggered,map_clear_regions_queued:map_clear_regions_queued,dynamic_clear_regions_queued:dynamic_clear_regions_queued,clear_regions_examined:clear.regions_examined,clear_nodes_examined:clear.nodes_examined,restored_nodes:clear.restored_nodes,historical_edges_restored:clear.historical_edges_restored,retained_cut_edges:clear.retained_cut_edges,clear_pending_regions:clear.pending_regions,clear_pending_nodes:clear.pending_nodes,coverage_attempts:coverage.sampling_attempts,coverage_added_nodes:coverage.sampling_added_nodes,coverage_pending_regions:coverage.sampling_pending_regions,reroot:reroot};
        metrics["people_changed"] = people_changed.into();
        metrics["people_metadata_only_refresh"] = people_metadata_only_refresh.into();
        metrics["people_added_or_moved"] = people_added_or_moved.into();
        metrics["people_removed_or_moved"] = people_removed_or_moved.into();
        metrics["people_clear_regions_queued"] = people_clear_regions_queued.into();
        Ok(self.output(v, metrics, started))
    }
    fn output(&mut self, v: &J, mut metrics: J, started: Instant) -> J {
        let full = self.checker(self.map.clone(), self.dynamic.clone(), self.people.clone(), None);
        let mut reason = "no_connected_path";
        let mut path = None;
        if !full.is_state_valid(&self.current) {
            reason = "current_blocked_or_out_of_bounds"
        } else if !full.is_state_valid(&self.goal) {
            reason = "goal_blocked_or_out_of_bounds"
        } else if let Some(mut candidate) = self.core.get_path() {
            if candidate.first() != Some(&self.current) {
                candidate.insert(0, self.current)
            }
            if candidate.last() == Some(&self.goal)
                && candidate
                    .windows(2)
                    .all(|w| full.is_edge_valid(&w[0], &w[1]))
            {
                reason = "candidate_path";
                path = Some(candidate)
            } else {
                reason = "path_failed_full_validation"
            }
        }
        let mut known_free = false;
        let path_json = if let Some(p) = &path {
            known_free = p
                .iter()
                .all(|s| full.vox.sweep(full.vox.xyz(s), full.vox.xyz(s), true))
                && p.windows(2).all(|w| {
                    full.vox
                        .sweep(full.vox.xyz(&w[0]), full.vox.xyz(&w[1]), true)
                });
            J::Array(p.iter().map(|p| json_state(p, &self.profile)).collect())
        } else {
            J::Null
        };
        let include_tree = v["include_tree"].as_bool().unwrap_or(true);
        let mut nodes = Vec::new();
        let mut edges = Vec::new();
        let mut invalid_edges = 0;
        let mut active = 0;
        let mut pruned = 0;
        for (id, n) in self.core.get_nodes().iter().enumerate() {
            if n.tree_id < 0 {
                pruned += 1;
            } else {
                active += 1;
            }
            if include_tree {
                nodes.push(json::object!{id:id,xyz:json_state(&n.state,&self.profile),tree_id:n.tree_id,parent:n.parent.map(J::from).unwrap_or(J::Null)});
            }
            if let Some(parent) = n.parent {
                let p = &self.core.get_nodes()[parent];
                if n.tree_id < 0
                    || p.tree_id != n.tree_id
                    || !full.is_edge_valid(&n.state, &p.state)
                {
                    invalid_edges += 1;
                } else if include_tree {
                    edges.push(json::array![id, parent]);
                }
            }
        }
        if invalid_edges > 0 {
            reason = "tree_invariant_failed";
        }
        self.path_seq += 1;
        metrics["nodes_total"] = self.core.get_nodes().len().into();
        metrics["nodes_active"] = active.into();
        metrics["nodes_pruned"] = pruned.into();
        metrics["invalid_tree_edges"] = invalid_edges.into();
        metrics["collision_checks_cumulative"] = self.counters.json();
        metrics["total_wall_ms"] = (started.elapsed().as_secs_f64() * 1000.).into();
        metrics["repair_budget_ms"] = self.repair_ms.into();
        metrics["prune_reroot_finalize_export_outside_repair_budget"] = true.into();
        json::object! {ok:true,protocol:"air_ground_smart3d.v1",robot_id:self.robot.clone(),session_id:self.session.clone(),tree_instance_id:self.instance,dimension:N,profile_id:self.profile.id.clone(),map:self.map.json_identity(),goal_revision:self.goal_revision,goal_xyz:json_state(&self.goal,&self.profile),current_xyz:json_state(&self.current,&self.profile),local_reaction_radius_m:LOCAL_REACTION_RADIUS_M,local_reaction_center_xyz:json_state(&self.current,&self.profile),sim_time:self.sim_time,dynamic_revision:self.dynamic_revision,dynamic_active_boxes:self.dynamic.len(),people_revision:self.people_revision,people_active_hazards:self.people.len(),people_hazards:J::Array(self.people.iter().map(|h| h.wire.clone()).collect()),path_id:format!("{}:{}:{}",self.session,self.goal_revision,self.path_seq),path_xyz:if invalid_edges>0{J::Null}else{path_json},path_status:reason,candidate_unknown_allowed:true,path_all_known_free:known_free,execution_authorized:false,tree_nodes:nodes,tree_edges:edges,tree_exported:include_tree,metrics:metrics}
    }
}
pub enum Worker {
    Empty,
    D2(Session<2>),
    D3(Session<3>),
}
impl Default for Worker {
    fn default() -> Self {
        Self::Empty
    }
}
impl Worker {
    pub fn handle(&mut self, v: &J) -> J {
        let result = (|| {
            if v["op"].as_str() == Some("init") {
                let started = Instant::now();
                let instance = match self {
                    Self::Empty => 1,
                    Self::D2(s) => s.instance + 1,
                    Self::D3(s) => s.instance + 1,
                };
                let mut next = match v["dimension"].as_u32() {
                    Some(2) => Self::D2(Session::init(v, instance)?),
                    Some(3) => Self::D3(Session::init(v, instance)?),
                    _ => return Err("dimension must be 2 or 3".into()),
                };
                let result = match &mut next {
                    Self::D2(s) => s.output(
                        v,
                        json::object! {operation:"init",initial_rrt:true},
                        started,
                    ),
                    Self::D3(s) => s.output(
                        v,
                        json::object! {operation:"init",initial_rrt:true},
                        started,
                    ),
                    _ => unreachable!(),
                };
                *self = next;
                Ok(result)
            } else {
                match self {
                    Self::Empty => Err("init_required".into()),
                    Self::D2(s) => s.call(v),
                    Self::D3(s) => s.call(v),
                }
            }
        })();
        result.unwrap_or_else(|reason:String|json::object!{ok:false,protocol:"air_ground_smart3d.v1",reason:reason,request_op:v["op"].clone(),robot_id:v["robot_id"].clone(),session_id:v["session_id"].clone(),execution_authorized:false})
    }
}
#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn clear_coverage_samples_true_z() {
        let v = json::object! {protocol:"air_ground_smart3d.v1",op:"init",robot_id:"r",session_id:"s",dimension:3,goal_revision:0,start:json::array![-0.7,0.,1.],goal:json::array![0.7,0.,1.],ranges:json::array![json::array![-3.,3.],json::array![-3.,3.],json::array![0.2,3.]],profile:json::object!{profile_id:"test",body_half_extents_m:json::array![0.1,0.1,0.1],body_offset_m:json::array![0.,0.,0.],planar_z:0.},map_snapshot:json::object!{scene_id:"test",scene_epoch:"1",content_version:"1",frame_id:"map",resolution:0.25,revision:0,capture_sim_time_watermark:0.,cells:json::array![]},initial_iterations:0};
        let mut s = Session::<3>::init(&v, 1).unwrap();
        let before = s.core.get_nodes().len();
        s.core
            .queue_static_clear_regions(&[(State::new([0., 0., 1.]), 2.)]);
        let result = s
            .core
            .maintain_static_clear_coverage(64, 8, 250000, Duration::from_secs(1));
        assert!(result.sampling_added_nodes > 0);
        assert!(s.core.get_nodes()[before..]
            .iter()
            .any(|n| (n.state.values()[2] - 1.).abs() > 0.1));
    }
}
#[cfg(test)]
mod budget_tests {
    use super::*;
    use smart_3d::rrt::{rrt_star, NearestNeighbors};
    use std::sync::atomic::{AtomicUsize, Ordering};
    struct Reject;
    impl ValidityChecker<f32, 2> for Reject {
        fn is_state_valid(&self, p: &State<2>) -> bool {
            *p.values() == [0., 0.] || *p.values() == [5., 0.]
        }
        fn is_edge_valid(&self, _: &State<2>, _: &State<2>) -> bool {
            false
        }
    }
    struct BadSamples(Arc<AtomicUsize>);
    impl SamplingDistribution<f32, 2> for BadSamples {
        fn sample(&mut self) -> State<2> {
            self.0.fetch_add(1, Ordering::Relaxed);
            State::new([1., 1.])
        }
    }
    #[test]
    fn wholly_rejected_sampling_returns_to_iteration_budget() {
        let goal = State::new([0., 0.]);
        let current = State::new([5., 0.]);
        let nodes = vec![rrt_star::Node::new(goal, None, 0.)];
        let mut nn = KdTreeNearestNeighbors::<f32, 2>::new();
        nn.add(goal, 0);
        let attempts = Arc::new(AtomicUsize::new(0));
        let mut core = SMART::new_from_initial_rrt_star_tree(
            &nodes,
            nn,
            current,
            goal,
            0.3,
            Box::new(Reject),
            Box::new(BadSamples(attempts.clone())),
            4.,
            1.5,
            1.5,
            8.,
            1.8,
            1.8,
            false,
            0.01,
        );
        core.initial_plan_build_path();
        core.update(
            current,
            0,
            &vec![],
            Some(&mut MaxIterationsTermination::new(1)),
        );
        assert_eq!(attempts.load(Ordering::Relaxed), 256);
        assert!(core.get_path().is_none());
    }
}
