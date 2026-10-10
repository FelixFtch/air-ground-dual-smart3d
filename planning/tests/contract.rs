use air_ground_smart3d_worker::{voxel::*, Worker};
use json::JsonValue as J;
use smart_3d::rrt::{RealVectorState, ValidityChecker};
use std::sync::Arc;
fn cell(k: [i32; 3], s: &str) -> J {
    json::object! {key:J::Array(k.into_iter().map(J::from).collect()),state:s,log_odds:if s=="occupied"{1.0}else{-1.0},last_observed_sim_time:1.0}
}
fn snapshot(cells: Vec<J>) -> J {
    json::object! {scene_id:"test",scene_epoch:"epoch1",content_version:"testv1",frame_id:"map",resolution:0.25,revision:0,capture_sim_time_watermark:1.0,cells:cells}
}
fn init(n: usize) -> J {
    let mut v = json::object! {protocol:"air_ground_smart3d.v1",op:"init",robot_id:"r",session_id:"s",dimension:n,goal_revision:0,profile:json::object!{profile_id:"test",body_half_extents_m:json::array![0.12,0.12,0.12],body_offset_m:json::array![0.,0.,0.],planar_z:0.349},map_snapshot:snapshot(vec![]),initial_iterations:350,initial_budget_ms:10000,repair_iterations:300,repair_budget_ms:300};
    if n == 2 {
        v["start"] = json::array![-2., 0.];
        v["goal"] = json::array![2., 0.];
        v["ranges"] = json::array![json::array![-3., 3.], json::array![-3., 3.]];
    } else {
        v["start"] = json::array![-2., 0., 1.];
        v["goal"] = json::array![2., 0., 1.];
        v["ranges"] = json::array![
            json::array![-3., 3.],
            json::array![-3., 3.],
            json::array![0.5, 3.]
        ];
    }
    v
}
fn call(op: &str, n: usize, rev: u64) -> J {
    json::object! {op:op,robot_id:"r",session_id:"s",goal_revision:rev,current:if n==2{json::array![-2.,0.]}else{json::array![-2.,0.,1.]}}
}
fn delta(base: u64, rev: u64, cells: Vec<J>) -> J {
    let mut d = snapshot(vec![]);
    d.remove("cells");
    d["base_revision"] = base.into();
    d["revision"] = rev.into();
    d["changed_cells"] = cells.into();
    d
}
fn assert_path(r: &J) {
    assert_eq!(r["ok"], true, "{}", r.dump());
    assert_eq!(r["metrics"]["invalid_tree_edges"], 0, "{}", r.dump());
    assert!(r["path_xyz"].is_array(), "{}", r.dump());
    assert_eq!(r["path_xyz"][0], r["current_xyz"]);
    assert_eq!(r["path_xyz"][r["path_xyz"].len() - 1], r["goal_xyz"]);
    assert_eq!(r["execution_authorized"], false);
}
#[test]
fn three_d_and_two_d_actual_current() {
    for n in [2, 3] {
        let mut w = Worker::default();
        let r = w.handle(&init(n));
        assert_path(&r);
        let mut q = call("plan", n, 0);
        q["current"] = if n == 2 {
            json::array![-1.6, 0.2]
        } else {
            json::array![-1.6, 0.2, 1.3]
        };
        let r = w.handle(&q);
        assert_path(&r);
        assert!((r["path_xyz"][0][0].as_f32().unwrap() + 1.6).abs() < 1e-5);
        if n == 2 {
            for p in r["path_xyz"].members() {
                assert!((p[2].as_f32().unwrap() - 0.349).abs() < 1e-6)
            }
        }
    }
}
#[test]
fn reroot_preserves_ids_and_edges_then_obstacles_clear() {
    for n in [2, 3] {
        let mut w = Worker::default();
        let before = w.handle(&init(n));
        assert_path(&before);
        let mut q = call("set_goal", n, 1);
        q["goal"] = if n == 2 {
            json::array![1.5, 1.5]
        } else {
            json::array![1.5, 1.5, 2.]
        };
        let changed = w.handle(&q);
        assert_path(&changed);
        assert_eq!(before["tree_instance_id"], changed["tree_instance_id"]);
        let old_count = before["tree_nodes"].len();
        assert_eq!(changed["metrics"]["reroot"]["retained_nodes"], old_count);
        assert_eq!(
            changed["metrics"]["reroot"]["retained_edges"],
            before["tree_edges"].len()
        );
        assert_eq!(
            changed["tree_nodes"][old_count]["xyz"],
            before["tree_nodes"][0]["xyz"]
        );
        for id in 1..old_count {
            assert_eq!(
                changed["tree_nodes"][id]["xyz"],
                before["tree_nodes"][id]["xyz"]
            );
        }
        let mut wall = Vec::new();
        for y in -2..=2 {
            for z in 0..=10 {
                wall.push(cell([0, y, z], "occupied"));
            }
        }
        let mut u = call("update", n, 1);
        u["map_delta"] = delta(0, 1, wall.clone());
        let blocked = w.handle(&u);
        assert_eq!(blocked["ok"], true);
        assert_eq!(blocked["metrics"]["invalid_tree_edges"], 0);
        assert_eq!(blocked["tree_instance_id"], before["tree_instance_id"]);
        if blocked["path_xyz"].is_array() {
            let map = Map::snapshot(&snapshot(wall.clone()), None).unwrap().0;
            let profile = Profile::parse(&init(n)["profile"]).unwrap();
            for p in blocked["path_xyz"].members() {
                let a = [
                    p[0].as_f32().unwrap(),
                    p[1].as_f32().unwrap(),
                    p[2].as_f32().unwrap(),
                ];
                let c = Checker::<3> {
                    map: Arc::new(map.clone()),
                    profile: profile.clone(),
                    ranges: [(-4., 4.); 3],
                    added_only: None,
                    counters: Arc::default(),
                };
                assert!(c.sweep(a, a, false));
            }
            for i in 1..blocked["path_xyz"].len() {
                let a = vector::<3>(&blocked["path_xyz"][i - 1], "a").unwrap();
                let b = vector::<3>(&blocked["path_xyz"][i], "b").unwrap();
                let c = Checker::<3> {
                    map: Arc::new(map.clone()),
                    profile: profile.clone(),
                    ranges: [(-4., 4.); 3],
                    added_only: None,
                    counters: Arc::default(),
                };
                assert!(c.sweep(a, b, false));
            }
        }
        for c in &mut wall {
            c["state"] = "free".into();
            c["log_odds"] = (-1).into();
        }
        u["map_delta"] = delta(1, 2, wall);
        let mut cleared = w.handle(&u);
        for _ in 0..5 {
            if cleared["path_xyz"].is_array() {
                break;
            }
            cleared = w.handle(&call("plan", n, 1));
        }
        assert_path(&cleared);
        assert_eq!(cleared["tree_instance_id"], before["tree_instance_id"]);
    }
}
#[test]
fn blocked_goal_and_revision_errors_are_transactional() {
    let mut w = Worker::default();
    assert_path(&w.handle(&init(3)));
    let mut u = call("update", 3, 0);
    u["map_delta"] = delta(0, 1, vec![cell([4, 4, 4], "occupied")]);
    assert_eq!(w.handle(&u)["ok"], true);
    let mut g = call("set_goal", 3, 1);
    g["goal"] = json::array![1.1, 1.1, 1.1];
    let r = w.handle(&g);
    assert_eq!(r["ok"], false);
    assert!(r["reason"].as_str().unwrap().contains("goal_blocked"));
    let state = w.handle(&call("state", 3, 0));
    assert_eq!(state["goal_revision"], 0);
    assert_eq!(state["map"]["revision"], 1);
    u["map_delta"] = delta(0, 2, vec![]);
    let r = w.handle(&u);
    assert_eq!(r["ok"], false);
    assert!(r["reason"].as_str().unwrap().contains("snapshot_required"));
    assert_eq!(w.handle(&call("state", 3, 0))["map"]["revision"], 1);
}
#[test]
fn dynamic_boxes_expire_without_polluting_map() {
    let mut w = Worker::default();
    assert_path(&w.handle(&init(3)));
    let mut q = call("update", 3, 0);
    q["sim_time"] = 2.into();
    q["dynamic_revision"] = 1.into();
    q["dynamic_boxes"] = json::array![
        json::object! {id:"partner",center_xyz:json::array![0.,0.,1.],half_extents_xyz:json::array![0.3,0.7,0.7],valid_until_sim_time:3.}
    ];
    let r = w.handle(&q);
    assert_eq!(r["ok"], true);
    assert_eq!(r["dynamic_active_boxes"], 1);
    assert_eq!(r["map"]["revision"], 0);
    assert_eq!(r["metrics"]["invalid_tree_edges"], 0);
    let mut q = call("plan", 3, 0);
    q["sim_time"] = 4.into();
    let r = w.handle(&q);
    assert_path(&r);
    assert_eq!(r["dynamic_active_boxes"], 0);
    assert_eq!(r["map"]["revision"], 0);
    assert_eq!(r["metrics"]["dynamic_changed"], true);
}
fn checker(cells: Vec<J>, half: [f32; 3]) -> Checker<3> {
    Checker {
        map: Arc::new(Map::snapshot(&snapshot(cells), None).unwrap().0),
        profile: Profile {
            id: "test".into(),
            half,
            offset: [0.; 3],
            planar_z: 0.,
        },
        ranges: [(-10., 10.); 3],
        added_only: None,
        counters: Arc::default(),
    }
}
#[test]
fn exact_swept_volume_catches_thin_between_endpoints_and_body_sides() {
    let c = checker(vec![cell([0, 0, 0], "occupied")], [0.1; 3]);
    let a = RealVectorState::new([-1., 0.1, 0.1]);
    let b = RealVectorState::new([1., 0.1, 0.1]);
    assert!(c.is_state_valid(&a) && c.is_state_valid(&b));
    assert!(!c.is_edge_valid(&a, &b));
    assert!(!c.sweep([-1., 0.33, 0.1], [1., 0.33, 0.1], false));
    assert!(c.sweep([-1., 0.36, 0.1], [1., 0.36, 0.1], false));
    assert!(c.sweep([-1., 0.1, 0.6], [1., 0.1, 0.6], false));
}
#[test]
fn go2_floor_below_body_is_kept_and_low_positive_obstacle_blocks() {
    let c = checker(vec![cell([0, 0, -1], "occupied")], [0.43, 0.43, 0.28]);
    assert!(c.sweep([0.1, 0.1, 0.349], [0.2, 0.1, 0.349], false));
    let c = checker(vec![cell([0, 0, 0], "occupied")], [0.43, 0.43, 0.28]);
    assert!(!c.sweep([0.1, 0.1, 0.349], [0.2, 0.1, 0.349], false));
}
#[test]
fn unknown_is_candidate_only_and_negative_keys_are_not_shifted() {
    let c = checker(vec![cell([-1, 0, 0], "occupied")], [0.02; 3]);
    assert!(!c.sweep([-0.1, 0.1, 0.1], [-0.1, 0.1, 0.1], false));
    assert!(c.sweep([1., 1., 1.], [2., 1., 1.], false));
    assert!(!c.sweep([1., 1., 1.], [2., 1., 1.], true));
    let mut free = Vec::new();
    for x in -1..=1 {
        for y in -1..=1 {
            for z in -1..=1 {
                free.push(cell([x, y, z], "free"));
            }
        }
    }
    let c = checker(free, [0.02; 3]);
    assert!(c.sweep([0.1; 3], [0.1; 3], true));
}
#[test]
fn compact_refresh_and_unknown_delete_apply() {
    let map = Map::snapshot(&snapshot(vec![cell([-1, 0, 0], "occupied")]), None)
        .unwrap()
        .0;
    let mut d = delta(0, 1, vec![]);
    d["capture_sim_time_watermark"] = 2.into();
    d["refreshes"] =
        json::array![json::object! {capture_sim_time:2.,keys:json::array![json::array![-1,0,0]]}];
    let (next, ch) = Map::delta(&d, &map).unwrap();
    assert!(ch.added.is_empty());
    assert_eq!(next.cells[&[-1, 0, 0]].observed, Some(2.));
    let d = delta(
        1,
        2,
        vec![json::object! {key:json::array![-1,0,0],state:"unknown"}],
    );
    let (next, ch) = Map::delta(&d, &next).unwrap();
    assert!(next.cells.is_empty());
    assert_eq!(ch.unknown_removed, 1);
    assert_eq!(ch.expired, vec![[-1, 0, 0]]);
}
#[test]
fn duplicate_keys_and_i32_min_rejected_without_panic() {
    assert!(Map::snapshot(
        &snapshot(vec![cell([0, 0, 0], "free"), cell([0, 0, 0], "occupied")]),
        None
    )
    .is_err());
    assert!(Map::snapshot(
        &snapshot(vec![json::parse(
            r#"{"key":[-2147483648,0,0],"state":"free","log_odds":-1,"last_observed_sim_time":1}"#
        )
        .unwrap()]),
        None
    )
    .is_err());
}

fn partner(center: [f32; 3], half: [f32; 3], until: f64) -> J {
    json::object! {id:"partner",center_xyz:J::Array(center.into_iter().map(J::from).collect()),half_extents_xyz:J::Array(half.into_iter().map(J::from).collect()),valid_until_sim_time:until}
}
#[test]
fn ttl_only_renewal_updates_lease_without_prune_clear_or_tree_change() {
    for n in [2, 3] {
        let mut w = Worker::default();
        let mut spec = init(n);
        spec["dynamic_revision"] = 1.into();
        spec["sim_time"] = 1.into();
        spec["dynamic_boxes"] = json::array![partner([0., 2.5, 1.], [0.2, 0.2, 0.3], 2.)];
        let initial = w.handle(&spec);
        assert_path(&initial);
        let mut request = call("update", n, 0);
        request["sim_time"] = 1.5.into();
        request["dynamic_revision"] = 2.into();
        request["dynamic_boxes"] = json::array![partner([0., 2.5, 1.], [0.2, 0.2, 0.3], 4.)];
        let renewed = w.handle(&request);
        assert_path(&renewed);
        assert_eq!(renewed["metrics"]["dynamic_changed"], false);
        assert_eq!(renewed["metrics"]["dynamic_ttl_only_refresh"], true);
        assert_eq!(renewed["metrics"]["prune_scan_triggered"], false);
        assert_eq!(renewed["metrics"]["native_map_result"], "CheckerRefreshed");
        assert_eq!(renewed["metrics"]["dynamic_clear_regions_queued"], 0);
        assert_eq!(renewed["metrics"]["map_clear_regions_queued"], 0);
        assert_eq!(renewed["metrics"]["clear_regions_examined"], 0);
        assert_eq!(renewed["tree_nodes"], initial["tree_nodes"]);
        assert_eq!(renewed["tree_edges"], initial["tree_edges"]);
        assert_eq!(renewed["map"]["revision"], 0);
        assert_eq!(renewed["dynamic_revision"], 2);
        // The new lease must actually survive the old expiration deadline.
        let mut later = call("plan", n, 0);
        later["sim_time"] = 3.into();
        let retained = w.handle(&later);
        assert_eq!(retained["dynamic_active_boxes"], 1);
        assert_eq!(retained["metrics"]["dynamic_changed"], false);
    }
}
#[test]
fn moved_resized_and_expired_boxes_keep_real_prune_and_recovery() {
    let mut w = Worker::default();
    let initial = w.handle(&init(3));
    assert_path(&initial);
    let mut q = call("update", 3, 0);
    q["sim_time"] = 2.into();
    q["dynamic_revision"] = 1.into();
    q["dynamic_boxes"] = json::array![partner([0., 0., 1.], [0.4, 0.7, 0.7], 20.)];
    let added = w.handle(&q);
    assert_eq!(added["ok"], true);
    assert_eq!(added["metrics"]["prune_scan_triggered"], true);
    assert_eq!(added["metrics"]["dynamic_added_or_moved_boxes"], 1);
    assert_eq!(added["metrics"]["dynamic_clear_regions_queued"], 0);
    assert!(added["metrics"]["nodes_pruned"].as_usize().unwrap() > 0);
    q["sim_time"] = 3.into();
    q["dynamic_revision"] = 2.into();
    q["dynamic_boxes"] = json::array![partner([0., 2., 1.], [0.4, 0.7, 0.7], 20.)];
    let moved = w.handle(&q);
    assert_eq!(moved["ok"], true);
    assert_eq!(moved["metrics"]["dynamic_changed"], true);
    assert_eq!(moved["metrics"]["prune_scan_triggered"], true);
    assert_eq!(moved["metrics"]["dynamic_clear_regions_queued"], 1);
    assert!(moved["metrics"]["restored_nodes"].as_usize().unwrap() > 0);
    assert_eq!(moved["metrics"]["invalid_tree_edges"], 0);
    q["sim_time"] = 4.into();
    q["dynamic_revision"] = 3.into();
    q["dynamic_boxes"] = json::array![partner([0., 2., 1.], [0.5, 0.8, 0.8], 20.)];
    let resized = w.handle(&q);
    assert_eq!(resized["ok"], true);
    assert_eq!(resized["metrics"]["dynamic_changed"], true);
    assert_eq!(resized["metrics"]["prune_scan_triggered"], true);
    assert_eq!(resized["metrics"]["dynamic_clear_regions_queued"], 1);
    let mut end = call("plan", 3, 0);
    end["sim_time"] = 21.into();
    let expired = w.handle(&end);
    assert_path(&expired);
    assert_eq!(expired["dynamic_active_boxes"], 0);
    assert_eq!(expired["metrics"]["dynamic_changed"], true);
    assert_eq!(expired["metrics"]["dynamic_clear_regions_queued"], 1);
    assert!(expired["metrics"]["restored_nodes"].as_usize().unwrap() > 0);
    assert_eq!(expired["tree_instance_id"], initial["tree_instance_id"]);
    assert_eq!(expired["map"]["revision"], 0);
}

#[test]
fn restored_null_times_keep_occupancy_without_certifying_free_and_refresh_normally() {
    let mut restored_free = cell([0,0,0], "free");
    restored_free["last_observed_sim_time"] = J::Null;
    let map = Map::snapshot(&snapshot(vec![restored_free.clone()]), None).unwrap().0;
    assert_eq!(map.cells[&[0,0,0]].observed, None);
    let c = checker(vec![restored_free], [0.02;3]);
    assert!(c.sweep([0.125;3],[0.125;3],false));
    assert!(!c.sweep([0.125;3],[0.125;3],true));
    let mut d = delta(0,1,vec![]);
    d["capture_sim_time_watermark"] = 2.0.into();
    d["refreshes"] = json::array![json::object!{capture_sim_time:2.,keys:json::array![json::array![0,0,0]]}];
    let (refreshed, changes) = Map::delta(&d, &map).unwrap();
    assert_eq!(refreshed.cells[&[0,0,0]].observed, Some(2.));
    assert!(changes.added.is_empty() && changes.cleared.is_empty());
    let mut valid = c.clone();valid.map=Arc::new(refreshed);
    assert!(valid.sweep([0.125;3],[0.125;3],true));
    for n in [2,3] {
        let mut obstacle=cell([0,0,if n==2{1}else{4}],"occupied");
        obstacle["last_observed_sim_time"]=J::Null;
        let c=checker(vec![obstacle.clone()],[0.12;3]);
        let z=if n==2{0.349}else{1.};
        assert!(!c.sweep([-2.,0.,z],[2.,0.,z],false));
        let mut v=init(n);v["map_snapshot"]=snapshot(vec![obstacle]);
        let mut worker=Worker::default();let accepted=worker.handle(&v);
        assert_eq!(accepted["ok"],true,"{}",accepted.dump());
        assert_eq!(accepted["metrics"]["invalid_tree_edges"],0);
        // Restored occupancy still rejects a goal inside the saved obstacle.
        v["goal"]=if n==2{json::array![0.125,0.125]}else{json::array![0.125,0.125,1.125]};
        let blocked=Worker::default().handle(&v);
        assert_eq!(blocked["ok"],false);
        assert_eq!(blocked["reason"],"goal_blocked_or_out_of_bounds");
    }
}
