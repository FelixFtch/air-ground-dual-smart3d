"""Human evidence updates must not reset the active motion transport."""
import json
import sim_io.runtime as module

def test_person_update_preserves_motion_transport(tmp_path,monkeypatch):
    runtime=module.Runtime.__new__(module.Runtime)
    runtime.scene={"scene_id":"test","person_prediction_enabled":True}
    runtime.epoch="epoch";runtime.root=tmp_path
    runtime.robots={"go2":{"instance":{"robot_config":{}}}}
    runtime.people_documents={};runtime.people_error=None
    runtime.motion=object();runtime.motion_rpc_latency_s=.02
    runtime.now_sim=lambda:1.
    (tmp_path/"shared").mkdir();(tmp_path/"shared/people.json").write_text(json.dumps({}))
    monkeypatch.setattr(module,"build_hazards",lambda *args:[])
    transport=runtime.motion
    assert runtime.person_hazards("go2")==[]
    assert runtime.motion is transport and runtime.motion_rpc_latency_s==.02
