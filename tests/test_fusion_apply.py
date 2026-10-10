"""One native frame may overlap one ordered apply; never a cache FIFO."""
import threading
import time
from types import SimpleNamespace

import pytest

from sim_io.fusion_apply import FusionFrame, OrderedApplyPipeline
from sim_io.runtime import Runtime


def test_next_native_work_overlaps_only_one_ordered_apply():
    started=threading.Event();release=threading.Event();second=threading.Event();native_done=threading.Event()
    order=[]
    def apply(number):
        order.append(('start',number))
        if number==1:
            started.set();assert release.wait(2)
        else:second.set()
        order.append(('finish',number))
    with OrderedApplyPipeline() as pipeline:
        pipeline.submit(apply,1);assert started.wait(1)
        def producer():
            native_done.set()  # Next RPC can complete while apply 1 is blocked.
            pipeline.submit(apply,2)  # Cannot queue apply 2 behind apply 1.
        producer_thread=threading.Thread(target=producer);producer_thread.start()
        assert native_done.wait(1);assert not second.is_set()
        release.set();producer_thread.join(2);assert not producer_thread.is_alive()
    assert order==[('start',1),('finish',1),('start',2),('finish',2)]


def test_worker_exception_surfaces_before_any_next_apply():
    calls=[]
    def fail():raise ValueError('bad delta')
    pipeline=OrderedApplyPipeline();pipeline.submit(fail)
    with pytest.raises(ValueError,match='bad delta'):pipeline.submit(lambda:calls.append(1))
    pipeline.close();assert calls==[]


def test_poll_surfaces_failure_even_when_no_new_sensor_frame_arrives():
    def fail():raise ValueError('failed while idle')
    with OrderedApplyPipeline() as pipeline:
        pipeline.submit(fail)
        # Ensure completion without consuming result; poll owns error delivery.
        assert isinstance(pipeline._future.exception(timeout=1),ValueError)
        with pytest.raises(ValueError,match='failed while idle'):pipeline.poll()


def test_shutdown_drains_and_is_idempotent():
    completed=[]
    pipeline=OrderedApplyPipeline();pipeline.submit(lambda:completed.append(7))
    pipeline.close();pipeline.close();assert completed==[7]
    with pytest.raises(RuntimeError,match='closed'):pipeline.submit(lambda:None)


def test_producer_error_still_drains_and_preserves_primary_exception():
    completed=[]
    with pytest.raises(ValueError,match='sensor RPC failure'):
        with OrderedApplyPipeline() as pipeline:
            pipeline.submit(lambda:completed.append(9))
            raise ValueError('sensor RPC failure')
    assert completed==[9]
    with pytest.raises(ValueError,match='producer first'):
        with OrderedApplyPipeline() as pipeline:
            def fail():raise RuntimeError('worker also failed')
            pipeline.submit(fail)
            raise ValueError('producer first')


def runtime_fixture():
    r=Runtime.__new__(Runtime)
    r.map_lock=threading.RLock();r.state_lock=threading.RLock();r.sensor_lock=threading.Lock()
    r.robots={'go2':{'sensor_fresh':{}},'uav':{'sensor_fresh':{}}}
    r.sensor_stats={('go2','lidar'):{'fused':0},('uav','lidar'):{'fused':0}}
    r.fusion_metrics={};r.map_watermark=0.;r.map_timing=[];r.sim_time=10.4;r.capture_sim_time=10.5
    r.display_index=SimpleNamespace(apply=lambda cells,delta:{'new_occupied':1,'cleared_occupied':0})
    r.cache=SimpleNamespace(cells={},apply=lambda delta:None)
    return r


def frame(robot='go2',number=23):
    now=time.monotonic()
    return FusionFrame(robot_id=robot,sensor_id='lidar',frame=number,capture_sim_time=10.,
        arrival_wall=now-.2,selected_wall=now-.18,request_started_wall=now-.1,response_wall=now-.03,
        prepare_s=.02,pose_lock_s=.01,rays_s=.05,points=3,free_rays=4,ray_coverage={'fixture':True},
        calibration_id='sensor-rig',sensor_to_world=[[1.]])


def test_runtime_freshness_and_counters_update_only_after_ordered_cache_commit():
    r=runtime_fixture();started=threading.Event();release=threading.Event();applied=[]
    def apply(delta):
        started.set();assert release.wait(2);applied.append(delta['revision'])
    r.cache.apply=apply
    frozen=frame();delta={'revision':8,'changed_cells':[],'_transport_timing':{'fixture':True}}
    with OrderedApplyPipeline() as pipeline:
        pipeline.submit(r._apply_fusion,delta,frozen,{})
        assert started.wait(1)
        assert r.robots['go2']['sensor_fresh']=={} and r.sensor_stats[('go2','lidar')]['fused']==0
        assert r.map_watermark==0. and r.fusion_metrics=={}
        release.set()
    assert applied==[8]
    fresh=r.robots['go2']['sensor_fresh']['lidar'];assert fresh['frame']==23 and fresh['sim_time']==10.
    assert r.robots['uav']['sensor_fresh']=={}
    assert r.sensor_stats[('go2','lidar')]['fused']==1 and r.map_watermark==10.
    metrics=r.fusion_metrics['go2/lidar']
    assert metrics['points']==3 and metrics['free_rays']==4 and metrics['capture_to_apply_sim_s']==.5
    assert metrics['map_request_s']==pytest.approx(.07)
    assert metrics['arrival_to_apply_wall_s']==pytest.approx(fresh['wall']-frozen.arrival_wall)
    assert metrics['apply_queue_s']>=.03 and metrics['cache_apply_s']>=0
    assert metrics['total_work_s']==pytest.approx(metrics['prepare_s']+metrics['pose_lock_s']+metrics['rays_s']+metrics['map_request_s']+metrics['apply_queue_s']+metrics['cache_apply_s'])


def test_rejected_delta_never_advances_freshness_or_counters():
    r=runtime_fixture()
    def fail(delta):raise ValueError('map_revision_gap')
    r.cache.apply=fail
    with pytest.raises(ValueError,match='revision_gap'):
        with OrderedApplyPipeline() as pipeline:pipeline.submit(r._apply_fusion,{'changed_cells':[]},frame(),{})
    assert not r.robots['go2']['sensor_fresh'] and not r.fusion_metrics
    assert r.map_watermark==0. and r.sensor_stats[('go2','lidar')]['fused']==0
