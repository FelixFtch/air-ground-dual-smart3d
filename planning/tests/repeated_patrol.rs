use air_ground_smart3d_worker::Worker;
use json::JsonValue as J;

// Long patrols revisit exactly the same four route coordinates. Every reroot
// retains the former root under a new node ID, so >32 observations per corner
// must remain legal nearest-neighbor data, in both the 2D and 3D planners.
#[test]
fn repeated_four_corner_patrol_keeps_tree_and_path_after_160_goal_changes() {
    for dimension in [2, 3] {
        let coordinate = |x: f32, y: f32| -> J {
            if dimension == 2 { json::array![x, y] } else { json::array![x, y, 2.] }
        };
        let corners = [(-8., -8.), (8., -8.), (8., 8.), (-8., 8.)];
        let ranges = if dimension == 2 {
            json::array![json::array![-10., 10.], json::array![-10., 10.]]
        } else {
            json::array![json::array![-10., 10.], json::array![-10., 10.], json::array![1., 3.]]
        };
        let mut worker = Worker::default();
        let initial = worker.handle(&json::object! {
            protocol:"air_ground_smart3d.v1", op:"init", robot_id:"r", session_id:"repeat",
            dimension:dimension, goal_revision:0, start:coordinate(-8., -8.), goal:coordinate(8., -8.),
            ranges:ranges, profile:json::object! {profile_id:"repeat", body_half_extents_m:json::array![0.12,0.12,0.12],body_offset_m:json::array![0.,0.,0.],planar_z:0.35},
            map_snapshot:json::object! {scene_id:"test",scene_epoch:"epoch",content_version:"test",frame_id:"map",resolution:0.25,revision:0,capture_sim_time_watermark:0.,cells:json::array![]},
            initial_iterations:80,initial_budget_ms:10000,repair_iterations:0,repair_budget_ms:1
        });
        assert_eq!(initial["ok"],true,"{}",initial.dump());
        let initial_count=initial["tree_nodes"].len();
        for revision in 1..=160 {
            let (x,y)=corners[(revision+1)%4];
            let r=worker.handle(&json::object! {op:"set_goal",robot_id:"r",session_id:"repeat",goal_revision:revision,
                current:coordinate(corners[revision%4].0,corners[revision%4].1),goal:coordinate(x,y)});
            assert_eq!(r["ok"],true,"dimension {dimension}, revision {revision}: {}",r.dump());
            assert_eq!(r["tree_instance_id"],initial["tree_instance_id"]);
            assert_eq!(r["tree_nodes"].len(),initial_count+revision);
            assert_eq!(r["metrics"]["invalid_tree_edges"],0);
            assert!(r["path_xyz"].is_array(),"dimension {dimension}, revision {revision}: {}",r.dump());
            assert_eq!(r["goal_xyz"][0],J::from(x));
            assert_eq!(r["goal_xyz"][1],J::from(y));
        }
    }
}
