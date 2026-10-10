use air_ground_smart3d_worker::{people, voxel::*, Worker};
use json::JsonValue as J;
use smart_3d::rrt::RealVectorState;

fn snapshot() -> J {
    json::object!{scene_id:"test",scene_epoch:"epoch1",content_version:"testv1",frame_id:"map",resolution:0.25,revision:0,capture_sim_time_watermark:1.,cells:json::array![]}
}
fn init(n:usize) -> J {
    json::object!{protocol:"air_ground_smart3d.v1",op:"init",robot_id:"r",session_id:"s",dimension:n,goal_revision:0,profile:json::object!{profile_id:"test",body_half_extents_m:json::array![0.12,0.12,0.12],body_offset_m:json::array![0.,0.,0.],planar_z:0.349},map_snapshot:snapshot(),initial_iterations:350,initial_budget_ms:10000,repair_iterations:128,repair_budget_ms:50,
    start:if n==2 {json::array![-2.,0.]}else{json::array![-2.,0.,1.]},goal:if n==2{json::array![2.,0.]}else{json::array![2.,0.,1.]},ranges:if n==2{json::array![json::array![-3.,3.],json::array![-3.,3.]]}else{json::array![json::array![-3.,3.],json::array![-3.,3.],json::array![0.5,3.]]}}
}
fn call(n:usize,now:f64)->J {
    json::object!{op:"plan",robot_id:"r",session_id:"s",goal_revision:0,sim_time:now,current:if n==2{json::array![-2.,0.]}else{json::array![-2.,0.,1.]}}
}
fn fixture(n:usize)->J {
    json::parse(include_str!("fixtures/people_geometry.json")).unwrap()[n-2].clone()
}
fn barrier(n:usize)->J {
    let mut h=fixture(n)["hazard"].clone();
    h["center_xyz"]=json::array![0.,0.,if n==2{0.349}else{1.}];
    h["axes_xyz"]=json::array![json::array![1.,0.,0.],json::array![0.,1.,0.],json::array![0.,0.,1.]];
    h["semi_axes_m"]=json::array![0.7,4.,if n==2{0.}else{4.}];
    h["velocity_xyz"]=json::array![0.,0.,0.];h["velocity_xy_valid"]=false.into();
    h["valid_until_sim_time"]=3.0.into(); h
}
#[test]
fn common_python_fixture_matches_native_2d_and_3d_narrowphase() {
    let map=Map::snapshot(&snapshot(),None).unwrap().0;
    for n in [2,3] {
        let f=fixture(n);let p=Profile::parse(&f["profile"]).unwrap();
        let h=json::array![f["hazard"].clone()];
        if n==2 {
            let parsed=people::parse::<2>(&h,1.,&map.identity,&p).unwrap();
            for c in f["cases"].members() {
                let a=vector::<3>(&c["start"],"a").unwrap();let b=vector::<3>(&c["end"],"b").unwrap();
                assert_eq!(parsed[0].geometry.intersects_edge(&RealVectorState::new([a[0],a[1]]),&RealVectorState::new([b[0],b[1]])),c["intersects"].as_bool().unwrap(),"{}",c.dump());
            }
        } else {
            let parsed=people::parse::<3>(&h,1.,&map.identity,&p).unwrap();
            for c in f["cases"].members() {
                let a=RealVectorState::new(vector::<3>(&c["start"],"a").unwrap());
                let b=RealVectorState::new(vector::<3>(&c["end"],"b").unwrap());
                assert_eq!(parsed[0].geometry.intersects_edge(&a,&b),c["intersects"].as_bool().unwrap(),"{}",c.dump());
            }
        }
    }
}
#[test]
fn temporary_people_prune_and_expire_recovering_the_same_smart_tree() {
    for n in [2,3] {
        let mut w=Worker::default();let before=w.handle(&init(n));
        assert_eq!(before["ok"],true,"{}",before.dump());assert!(before["path_xyz"].is_array());
        assert_eq!(before["local_reaction_radius_m"],4.);
        assert_eq!(before["local_reaction_center_xyz"],before["current_xyz"]);
        let mut q=call(n,2.);q["people_hazards"]=json::array![barrier(n)];
        let blocked=w.handle(&q);
        assert_eq!(blocked["ok"],true,"{}",blocked.dump());
        assert_eq!(blocked["tree_instance_id"],before["tree_instance_id"]);
        assert_eq!(blocked["people_active_hazards"],1);
        assert_eq!(blocked["metrics"]["people_changed"],true);
        assert_eq!(blocked["metrics"]["invalid_tree_edges"],0);
        assert!(blocked["path_xyz"].is_null());
        assert!(blocked["metrics"]["nodes_pruned"].as_usize().unwrap()>0);
        // Metadata and lease renewal do not repeatedly prune or clear geometry.
        q["sim_time"]=2.1.into();q["people_hazards"][0]["valid_until_sim_time"]=5.0.into();
        let renewed=w.handle(&q);
        assert_eq!(renewed["ok"],true,"{}",renewed.dump());
        assert_eq!(renewed["metrics"]["people_changed"],false);
        assert_eq!(renewed["metrics"]["people_metadata_only_refresh"],true);
        assert_eq!(renewed["metrics"]["prune_scan_triggered"],false);
        assert_eq!(renewed["metrics"]["people_clear_regions_queued"],0);
        assert_eq!(renewed["people_revision"],blocked["people_revision"]);
        // Omitted layer expires by simulation time; actual map stays unchanged.
        let expired=w.handle(&call(n,6.));
        assert_eq!(expired["ok"],true,"{}",expired.dump());
        assert_eq!(expired["people_active_hazards"],0);
        assert_eq!(expired["metrics"]["people_clear_regions_queued"],1);
        assert_eq!(expired["metrics"]["invalid_tree_edges"],0);
        assert_eq!(expired["map"]["revision"],0);
        assert_eq!(expired["tree_instance_id"],before["tree_instance_id"]);
        let mut recovered=expired;
        for _ in 0..10 {if recovered["path_xyz"].is_array(){break;} recovered=w.handle(&call(n,6.));}
        assert!(recovered["path_xyz"].is_array(),"{}",recovered.dump());
        for i in 0..before["tree_nodes"].len() {
            assert_eq!(before["tree_nodes"][i]["xyz"],recovered["tree_nodes"][i]["xyz"]);
        }
    }
}
#[test]
fn native_rejects_identity_malformed_expired_and_unknown_vertical_velocity_transactionally() {
    let mut w=Worker::default();assert_eq!(w.handle(&init(3))["ok"],true);
    for (key,value) in [("scene_epoch",J::from("other")),("scene_id",J::from("other")),
        ("profile_id",J::from("other")),("valid_until_sim_time",J::from(1.)),
        ("position_reference_sim_time",J::from(10.)),("velocity_z_valid",J::from(true)),
        ("semi_axes_m",json::array![0.,1.,1.]),("axes_xyz",json::array![]),
        ("footprint_included",J::from(false))] {
        let mut q=call(3,2.);let mut h=barrier(3);h[key]=value;q["people_hazards"]=json::array![h];
        let r=w.handle(&q);assert_eq!(r["ok"],false,"{}",r.dump());
        let mut state=call(3,1.);state["op"]="state".into();let unchanged=w.handle(&state);
        assert_eq!(unchanged["people_active_hazards"],0);assert_eq!(unchanged["sim_time"],1.);
    }
}

#[test]
fn native_rejects_heading_that_disagrees_with_valid_speed() {
    let map=Map::snapshot(&snapshot(),None).unwrap().0;
    let f=fixture(3);let p=Profile::parse(&f["profile"]).unwrap();
    let mut h=f["hazard"].clone();
    h["axes_xyz"]=json::array![json::array![1.,0.,0.],json::array![0.,1.,0.],json::array![0.,0.,1.]];
    let error=people::parse::<3>(&json::array![h],1.,&map.identity,&p).unwrap_err();
    assert!(error.contains("heading axis"));
}
