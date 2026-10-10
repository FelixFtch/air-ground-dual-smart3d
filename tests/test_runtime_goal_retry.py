"""Blocked-goal recovery keeps native map transactions and bounded mapper I/O."""
import threading
from types import SimpleNamespace

import pytest

from mapping.client import MapError
import sim_io.runtime as module
from sim_io.runtime import Runtime


class Stop:
    def __init__(self):
        self.stopped = False
        self.waits = []
        self.after_wait = lambda: None

    def is_set(self):
        return self.stopped

    def wait(self, seconds):
        self.waits.append(seconds)
        assert len(self.waits) < 10, "Planner retry did not terminate in bounded fixture"
        self.after_wait()


class Mapper:
    def __init__(self, history_gap=False):
        self.calls = []
        self.history_gap = history_gap

    def request(self, op, **fields):
        self.calls.append((op, fields))
        if op == "snapshot":
            # Only startup and an actual expired compact history may fetch this.
            snapshots = sum(kind == "snapshot" for kind, _ in self.calls)
            assert snapshots <= (2 if self.history_gap else 1), "Unnecessary full-map snapshot"
            return {"revision": 6 if snapshots == 1 else 8, "cells": []}
        assert op == "changes" and fields["compact"] is True
        base = fields["since_revision"]
        if self.history_gap and base == 6:
            raise MapError({"error": "snapshot_required", "message": "Fixture history expired"})
        return {"base_revision": base, "revision": 8 if base == 6 else base+1,
                "changed_cells": [], "refreshes": []}


def fixture(tmp_path, monkeypatch, *, history_gap=False, recovery_failure=None):
    runtime = Runtime.__new__(Runtime)
    runtime.epoch = "retry-fixture"
    runtime.root = tmp_path
    runtime.stop = Stop()
    runtime.map_lock = threading.RLock()
    runtime.state_lock = threading.RLock()
    runtime.cache = SimpleNamespace(revision=6)
    runtime.sim_time = runtime.capture_sim_time = 10.
    runtime.scene = {"bounds": [[-5, 5], [-5, 5], [-1, 3]], "person_prediction_enabled": True}
    runtime.team = {"planning": {"initial_iterations_2d": 1, "repair_iterations": 1, "period_s": .4}}
    cfg = {"dimension": 2, "profile_id": "fixture", "body_half_extents_m": [.1]*3,
           "body_offset_m": [0.]*3, "planar_z": .33}
    state = {"instance": {"robot_config": cfg, "route": [[1., 1., .33], [2., 2., .33]]},
             "position": [0., 0., .33], "route_index": 0, "goal_revision": 1,
             "sensor_fresh": {"lidar": {"sim_time": 10.}}, "planner": {}, "speed": 0.}
    runtime.robots = {"r": state}
    runtime.mapper = Mapper(history_gap)
    events = []
    runtime.event = lambda kind, **fields: events.append((kind, fields))
    runtime.dynamic_boxes = lambda rid, stamp: [{"fixture_reference_time": stamp}]
    runtime.person_hazards = lambda rid, stamp: [{"fixture_reference_time": stamp}]

    class Worker:
        def __init__(self, *args, **kwargs):
            self.calls = []
            self.accepted_map_revision = 0
            self.accepted_goal_revision = 0
            self.closed = False

        def call(self, request):
            self.calls.append(request)
            if request["op"] == "init":
                self.accepted_map_revision = request["map_snapshot"]["revision"]
                self.accepted_goal_revision = request["goal_revision"]
            else:
                if "map_delta" in request:
                    assert request["map_delta"]["base_revision"] == self.accepted_map_revision
                    revision = request["map_delta"]["revision"]
                else:
                    revision = request["map_snapshot"]["revision"]
                if request["op"] == "set_goal":
                    assert request["goal_revision"] == 2
                    # Simulate new robot observations while a blocked goal is
                    # considered. Rejection commits neither map nor goal.
                    runtime.capture_sim_time += .2
                    state["position"][0] += .1
                    return {"ok": False, "reason": "goal_blocked_or_out_of_bounds"}
                assert request["goal_revision"] == self.accepted_goal_revision == 1
                if recovery_failure == "rejected":
                    return {"ok": False, "reason": "invalid_hazard_contract"}
                self.accepted_map_revision = revision
            result = {"ok": True, "map": {"revision": self.accepted_map_revision},
                      "path_status": "fixture", "path_xyz": [], "tree_instance_id": "same-tree"}
            if not (request["op"] == "update" and recovery_failure == "missing_capability"):
                result.update(people_revision=0, people_active_hazards=0, people_hazards=[])
            return result

        def close(self):
            self.closed = True

    worker = Worker()
    monkeypatch.setattr(module, "JsonProcess", lambda *a, **kw: worker)
    def after_wait():
        if state["goal_revision"] == 1:
            state["goal_revision"], state["route_index"] = 2, 1
        elif sum(request["op"] == "update" for request in worker.calls) == 2:
            runtime.stop.stopped = True
    runtime.stop.after_wait = after_wait
    return runtime, state, worker, events


@pytest.mark.parametrize("history_gap", [False, True])
def test_blocked_goal_reuses_unapplied_map_and_compact_catchup(tmp_path, monkeypatch, history_gap):
    runtime, state, worker, events = fixture(tmp_path, monkeypatch, history_gap=history_gap)
    runtime.planner_loop("r")
    assert "planner_error" not in state
    assert [request["op"] for request in worker.calls] == ["init", "set_goal", "update", "set_goal", "update"]
    assert sum(op == "snapshot" for op, _ in runtime.mapper.calls) == (2 if history_gap else 1)
    changes = [fields for op, fields in runtime.mapper.calls if op == "changes"]
    assert [request["since_revision"] for request in changes] == [6, 8]
    for rejected, recovered in ((worker.calls[1], worker.calls[2]), (worker.calls[3], worker.calls[4])):
        field = "map_delta" if "map_delta" in rejected else "map_snapshot"
        assert recovered[field] is rejected[field]
        assert ("map_delta" in recovered) != ("map_snapshot" in recovered)
        assert recovered["goal_revision"] == 1 and "goal" not in recovered
        assert recovered["sim_time"] > rejected["sim_time"]
        assert recovered["current"][0] > rejected["current"][0]
        assert recovered["dynamic_revision"] > rejected["dynamic_revision"]
        assert recovered["dynamic_boxes"][0]["fixture_reference_time"] == recovered["sim_time"]
        assert recovered["people_hazards"][0]["fixture_reference_time"] == recovered["sim_time"]
    assert worker.accepted_map_revision == 9
    assert worker.accepted_goal_revision == 1
    assert state["planner"]["path_xyz"] is None  # The previous goal is never executable.
    assert state["planner"]["goal_revision"] == 1 and state["goal_revision"] == 2
    assert state["planner"]["tree_instance_id"] == "same-tree"
    assert state["planner"]["map"]["revision"] == 9
    assert worker.closed


@pytest.mark.parametrize("failure,reason", [("rejected", "planner_recovery_rejected: invalid_hazard_contract"),
                                           ("missing_capability", "worker_missing_people_hazard_capability")])
def test_recovery_contract_failure_stops_instead_of_retrying_or_fetching_snapshot(tmp_path, monkeypatch, failure, reason):
    runtime, state, worker, events = fixture(tmp_path, monkeypatch, recovery_failure=failure)
    runtime.planner_loop("r")
    assert state["status"] == "planner_error" and reason in state["planner_error"]
    assert state["planner"] == {} and state["speed"] == 0
    assert [request["op"] for request in worker.calls] == ["init", "set_goal", "update"]
    assert sum(op == "snapshot" for op, _ in runtime.mapper.calls) == 1
    assert events[-1][0] == "planner_failed"
    assert worker.closed


@pytest.mark.parametrize("panic_during_blocked_goal_recovery", [False, True])
def test_native_panic_reinitializes_only_planner_from_current_evidence(tmp_path, monkeypatch, panic_during_blocked_goal_recovery):
    runtime, state, _, events = fixture(tmp_path, monkeypatch)
    map_calls=[]
    def map_request(op, **fields):
        map_calls.append((op, fields))
        if op=='snapshot':
            return {'revision':6 if len(map_calls)==1 else 10,'cells':[]}
        assert op=='changes'
        return {'base_revision':fields['since_revision'],'revision':9,'changed_cells':[],'refreshes':[]}
    runtime.mapper.request=map_request
    calls=[]
    panicked=False
    class PanicWorker:
        closed=False
        def call(self, request):
            nonlocal panicked
            calls.append(request)
            if request['op']=='set_goal':
                return {'ok':False,'reason':'goal_blocked_or_out_of_bounds'}
            if request['op']=='update' and not panicked:
                panicked=True
                state['position']=[.4,.2,.33]
                runtime.capture_sim_time=11.
                return {'ok':False,'reason':'native_panic_session_invalidated'}
            assert request['op']=='init'
            if panicked:
                assert state['planner']=={} and state['speed']==0
                assert request['current']==[.4,.2] and request['sim_time']==11.
                assert request['goal']==[2.,2.] and request['goal_revision']==2
                assert request['session_id'].endswith('-recovery1')
                assert request['map_snapshot']['revision']==10
                runtime.stop.stopped=True
            return {'ok':True,'map':{'revision':request['map_snapshot']['revision']},
                    'path_status':'fixture','path_xyz':[[.4,.2,.33],[2.,2.,.33]],
                    'tree_instance_id':1,'people_revision':0,'people_active_hazards':0,'people_hazards':[]}
        def close(self):self.closed=True
    worker=PanicWorker()
    monkeypatch.setattr(module,'JsonProcess',lambda *a,**kw:worker)
    waits=0
    def after_wait():
        nonlocal waits
        waits+=1
        if waits==1 and panic_during_blocked_goal_recovery:
            state['goal_revision'],state['route_index']=2,1
        elif panicked:
            state['goal_revision'],state['route_index']=2,1
    runtime.stop.after_wait=after_wait
    runtime.planner_loop('r')
    assert 'planner_error' not in state
    assert state['planner_recoveries']==1 and state['planner']['goal_revision']==2
    assert state['planner']['path_xyz'] and worker.closed
    assert [x['op'] for x in calls]==(['init','set_goal','update','init'] if panic_during_blocked_goal_recovery else ['init','update','init'])
    assert [x for x,_ in map_calls].count('snapshot')==2
    assert not any(x in ('init','clear','load','save') for x,_ in map_calls)
    assert any(e[0]=='planner_session_invalidated' and e[1]['shared_map_preserved'] for e in events)
