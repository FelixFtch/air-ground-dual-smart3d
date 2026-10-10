"""One scene owner; independent robot planners, one shared probabilistic OcTree.

CARLA actor truth is used only for robot odometry/cooperative robot constraints.
Human obstacles reach the map through sensor measurements, never actor queries.
"""
from __future__ import annotations
import argparse
import gc
from collections import OrderedDict
from sim_io import json_codec as json
import math
import os
from pathlib import Path
import signal
import sys
import threading
import time
import traceback
import uuid
import numpy as np
import carla

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT))
from sim_io.config import carla_port, load_team, runtime_home
from sim_io.restart_state import load_restart_state
from sim_io.geometry import to_map_xyz,to_carla_xyz,transform_points,depth_points
from sim_io.ipc import JsonProcess
from sim_io.motion_rpc import MotionClient
from sim_io.people import People
from sim_io.fusion_schedule import choose_candidate
from sim_io.fusion_apply import FusionFrame, OrderedApplyPipeline
from sim_io.lidar_rays import prepare_lidar_frame, observed_xy_azimuth_span_deg
from mapping.client import OctomapClient,MapCache,MapError
from execution.guard import check_path_segment
from execution.path_follow import lookahead,bounded_step
from execution.recovery import find_recovery_target,check_recovery_segment
from observability.runtime_snapshot import MapDisplayIndex
from observability.tree_snapshots import TreeSnapshotWriter
from perception.hazards import build_hazards,check_hazard_segment


def atomic_json(path,data):
    path=Path(path);path.parent.mkdir(parents=True,exist_ok=True)
    tmp=path.with_suffix('.tmp');tmp.write_text(json.dumps(data,allow_nan=False,separators=(',',':')));tmp.replace(path)


class Runtime:
    def __init__(self,team,resume_map=None,resume_state=None):
        if resume_state and not resume_map:
            raise ValueError('resume_state requires explicit resume_map')
        self.restart_states=load_restart_state(team,resume_state) if resume_state else {}
        # Streaming JSON creates many short-lived coordinate arrays. Collect in
        # bounded larger batches; normal cyclic GC stays enabled in this process.
        gc.set_threshold(7000,10,10)
        self.team=team;self.scene=team['scene_config'];self.epoch=uuid.uuid4().hex
        if os.environ.get('AIR_GROUND_CARLA_PORT'):self.scene['carla_port']=carla_port()
        self.root=runtime_home()/'runs'/self.epoch;self.root.mkdir(parents=True)
        self.tree_snapshots=TreeSnapshotWriter(self.root,self.epoch)
        self.events=open(self.root/'events.jsonl','a',buffering=1)
        self.stop=threading.Event();self.sensor_lock=threading.Lock();self.map_lock=threading.RLock();self.state_lock=threading.RLock()
        self.latest={};self.rgb_history={};self.pose_history=OrderedDict();self.sensor_stats={};self.threads=[];self.robots={};self.sim_time=0.;self.last_frame=None
        self.motion=None;self.motion_rpc_latency_s=None;self.people_error=None;self.people_documents={};self.capture_sim_time=0.;self.map_watermark=0.;self.map_timing=[];self.map_error=None;self.fatal=None;self.people=None;self.tick_callback_id=None
        self.client=carla.Client(self.scene['carla_host'],self.scene['carla_port']);self.client.set_timeout(10)
        self.world=self.client.get_world();self.world_id=self.world.id
        if not self.world.get_map().name.endswith('/AirGroundLab_Main'):raise RuntimeError('wrong independent scene')
        if self.world.get_settings().synchronous_mode:raise RuntimeError('requires asynchronous world')
        existing=[a.id for a in self.world.get_actors() if a.attributes.get('role_name','').startswith('ag_')]
        if existing:raise RuntimeError('owned roles already exist; stop old runtime before retry: '+str(existing))
        self.mapper=OctomapClient(ROOT/'mapping/build/octomap_service',profile_transport=True)
        b=self.scene['bounds'];self.mapper.request('init',scene_id=self.scene['scene_id'],scene_epoch=self.epoch,
            content_version=self.scene['content_version'],frame_id=self.scene['frame_id'],resolution=self.scene['resolution'],
            bounds={'min':[v[0] for v in b],'max':[v[1] for v in b]},out_of_order_window_s=.35,history_limit=128,
            **self.scene.get('occupancy_sensor_model',{}))
        if resume_map:
            self.mapper.request('load',path=str(Path(resume_map).resolve()))
            self.event('map_restored',path=str(resume_map),fresh_sensor_evidence_required=True)
        self.cache=MapCache(self.mapper.request('snapshot'))
        ground=next(i['robot_config'] for i in team['robots'] if i['robot_config']['dimension']==2)
        self.display_index=MapDisplayIndex(self.cache.cells,self.scene['resolution'],ground['planar_z'],ground['body_half_extents_m'][2],ground['body_offset_m'][2])
        self.fusion_metrics={};self.publish_timing=[];self.gc_metrics={"count":0,"total_s":0.,"max_s":0.};self.gc_started={}
        gc.callbacks.append(self.record_gc)
        atomic_json(self.root/'resolved_profiles.json',team)
        atomic_json(runtime_home()/'latest_run.json',{'run_root':str(self.root),'epoch':self.epoch,'pid':os.getpid(),'started_wall':time.time()})

    def record_gc(self,phase,info):
        generation=info["generation"]
        if phase=="start":self.gc_started[generation]=time.monotonic()
        elif generation in self.gc_started:
            elapsed=time.monotonic()-self.gc_started.pop(generation)
            self.gc_metrics["count"]+=1;self.gc_metrics["total_s"]+=elapsed;self.gc_metrics["max_s"]=max(self.gc_metrics["max_s"],elapsed)

    def event(self,kind,**data):
        self.events.write(json.dumps({'wall_time':time.time(),'sim_time':self.sim_time,'kind':kind,**data},default=str)+'\n')

    def spawn(self):
        library=self.world.get_blueprint_library();origin=self.scene['carla_world_origin_m']
        for instance in self.team['robots']:
            rid=instance['robot_id'];cfg=instance['robot_config'];bp=library.find(cfg['blueprint']);bp.set_attribute('role_name',instance['role_name'])
            if cfg['dimension']==3:
                values={'visual_scale':instance['appearance_config']['visual_scale'],'visual_yaw_degrees':instance['appearance_config'].get('visual_yaw_degrees',-90),'body_half_x_m':.48,'body_half_y_m':.48,'body_half_z_m':.34,
                    'max_speed_mps':cfg['max_speed_mps'],'max_acceleration_mps2':2.,'max_yaw_rate_rps':cfg['max_yaw_rate_rps'],
                    'max_yaw_acceleration_rps2':2.,'max_request_translation_m':.15}
                # UE bounds constrain the whole body; profile bounds describe allowed centers.
                fb=self.scene['flight_bounds'];physical=[.48,.48,.34]
                for k,axis in enumerate('xyz'):
                    lo,hi=fb[k]
                    if axis=='y':lo,hi=-hi,-lo
                    if axis=='z':lo+=origin[2];hi+=origin[2]
                    values['flight_min_'+axis+'_m']=lo-physical[k];values['flight_max_'+axis+'_m']=hi+physical[k]
                for key,value in values.items():bp.set_attribute(key,str(value))
            restored=self.restart_states.get(rid)
            initial_position=restored['position'] if restored else instance['spawn_map_xyz']
            initial_yaw=restored['yaw'] if restored else instance['yaw']
            pos=to_carla_xyz(initial_position,origin)
            actor=self.world.spawn_actor(bp,carla.Transform(carla.Location(*pos),carla.Rotation(yaw=-math.degrees(initial_yaw))))
            state={'instance':instance,'actor':actor,'sensors':[],'sensor_definitions':{},'position':list(instance['spawn_map_xyz']),'yaw':instance['yaw'],
                   'route_index':1 if math.dist(instance['spawn_map_xyz'],instance['route'][0])<cfg['goal_tolerance_m'] and len(instance['route'])>1 else 0,
                   'goal_revision':1,'planner':{},'worker':None,'status':'mapping_warmup','distance_m':0.,'vertical_distance_m':0.,
                   'speed':0.,'actual_speed':0.,'completed_goals':0,'last_control_error':None,'last_plan_wall':0.,'sensor_fresh':{},'motion_block_count':0}
            if restored:
                state.update(restored)
                self.event('robot_pose_restored',robot_id=rid,position=state['position'],yaw=state['yaw'],
                    route_index=state['route_index'],completed_goals=state['completed_goals'])
            self.robots[rid]=state
            sc=instance['sensors_config']
            definitions=[('lidar',sc['lidar']),('rgb',sc['camera']),('depth',sc['camera'])]
            definitions += [('depth_aux_'+str(i),c) for i,c in enumerate(sc.get('clearance_depth',[]))]
            for kind,c in definitions:
                state['sensor_definitions'][kind]=c
                sb=library.find('sensor.lidar.ray_cast' if kind=='lidar' else 'sensor.camera.'+('depth' if kind.startswith('depth') else 'rgb'))
                attrs={'sensor_tick':c['sensor_tick']}
                if kind=='lidar':attrs.update(channels=c['channels'],points_per_second=c['points_per_second'],rotation_frequency=c['rotation_frequency'],range=c['range_m'],upper_fov=c['upper_fov'],lower_fov=c['lower_fov'],noise_stddev=c['noise_stddev'],dropoff_general_rate=0.,dropoff_zero_intensity=0.)
                else:attrs.update(image_size_x=c['width'],image_size_y=c['height'],fov=c['fov'])
                if kind=='rgb':
                    for key in ('post_process_profile','use_ray_tracing','enable_postprocess_effects'):
                        if key in c and sb.has_attribute(key):attrs[key]=c[key]
                for key,value in attrs.items():sb.set_attribute(key,str(value).lower() if isinstance(value,bool) else str(value))
                local=carla.Transform(carla.Location(*c['mount_m']),carla.Rotation(pitch=c.get('pitch_degrees',0),yaw=c.get('yaw_degrees',0)))
                sensor=self.world.spawn_actor(sb,local,attach_to=actor,attachment_type=carla.AttachmentType.Rigid)
                sensor.listen(lambda measure,rid=rid,kind=kind:self.sensor_callback(rid,kind,measure))
                state['sensors'].append((kind,sensor,np.array(local.get_matrix(),dtype=float)))
            self.event('robot_spawned',robot_id=rid,actor_id=actor.id,profile=cfg['profile_id'])
        self.tick_callback_id=self.world.on_tick(self.capture_robot_poses)
        self.people=People(self.world,self.scene['human_count'],self.scene['human_speed_mps'])
        self.event('people_spawned',count=len(self.people.actors),truth_used_for_perception=False)

    def capture_robot_poses(self,snapshot):
        self.capture_sim_time=snapshot.timestamp.elapsed_seconds
        history={}
        for rid,s in self.robots.items():
            a=snapshot.find(s['actor'].id)
            if a:
                p=a.get_transform().location
                history[rid]=to_map_xyz([p.x,p.y,p.z],self.scene['carla_world_origin_m']).tolist()
        with self.state_lock:
            self.pose_history[snapshot.frame]=history
            while len(self.pose_history)>180:self.pose_history.popitem(last=False)

    def sensor_callback(self,rid,kind,m):
        with self.sensor_lock:
            key=(rid,kind);previous=self.latest.get(key)
            if kind=='rgb':
                buffer=self.rgb_history.setdefault(rid,OrderedDict());buffer[m.frame]=m
                while len(buffer)>20:buffer.popitem(last=False)
            self.latest[key]=(m,time.monotonic())
            if previous:self.sensor_stats.setdefault(key,{'received':0,'fused':0,'replaced':0})['replaced']+=1
            self.sensor_stats.setdefault(key,{'received':0,'fused':0,'replaced':0})['received']+=1

    def mapping_loop(self):
        fused_frames={};last_rgb={};last_served={}
        try:
            with OrderedApplyPipeline() as apply_pipeline:
                while not self.stop.is_set():
                    apply_pipeline.poll()
                    with self.sensor_lock:
                        candidates=[(float(m.timestamp),rid,kind,m,arrival) for (rid,kind),(m,arrival) in self.latest.items()
                                    if kind!='rgb' and fused_frames.get((rid,kind))!=m.frame]
                    if not candidates:self.stop.wait(.015);continue
                    # Use latest measurements and bounded due-time fairness; never build
                    # an old-frame queue or starve depth when LiDAR is busy.
                    now=time.monotonic()
                    selected=choose_candidate(candidates,last_served,now,
                        lambda robot,kind:self.robots[robot]['sensor_definitions'][kind].get('fusion_period_s',.2 if kind=='lidar' else .5))
                    if selected is None:self.stop.wait(.005);continue
                    _,rid,kind,m,arrival=selected;key=(rid,kind);fused_frames[key]=m.frame;last_served[key]=now
                    if self.sim_time-m.timestamp>1.5:continue
                    state=self.robots[rid];cfg=state['instance']['robot_config'];sc=state['instance']['sensors_config'];sensor_cfg=state['sensor_definitions'][kind]
                    extrinsic=next(t for k,_,t in state['sensors'] if k==kind)
                    sensor_world=np.array(m.transform.get_matrix(),dtype=float)
                    max_range=sensor_cfg.get('effective_mapping_range_m',sc['lidar']['effective_mapping_range_m'])
                    if kind=='lidar':
                        local=np.frombuffer(m.raw_data,dtype=np.float32).reshape(-1,4)[:,:3].astype(float)
                        raw_return_span=observed_xy_azimuth_span_deg(local)
                        raw_return_count=len(local)
                        # All real return voxels remain eligible; ray preparation below
                        # separates near marking from far measured-space clearing.
                        free=np.empty((0,3));far=np.empty(0,dtype=bool)
                    else:local,free,far=depth_points(m,sensor_cfg['fov'],stride=sensor_cfg.get('mapping_stride',20),max_range=max_range)
                    local=self.remove_body_hits(local,extrinsic,cfg)
                    points=transform_points(local,sensor_world,self.scene['carla_world_origin_m'])
                    # Partner robot body is a temporary cooperative constraint; never make it a permanent wall.
                    before_pose_lock=time.monotonic()
                    with self.state_lock:history=self.pose_history.get(m.frame)
                    after_pose_lock=time.monotonic()
                    if not history or len(history)!=len(self.robots):
                        if time.monotonic()-arrival<.6:fused_frames.pop(key,None)
                        else:self.event('missing_capture_pose',robot_id=rid,sensor=kind,frame=m.frame)
                        self.stop.wait(.005)
                        continue
                    if history:
                        for other,pose in history.items():
                            if other==rid:continue
                            ocfg=self.robots[other]['instance']['robot_config'];half=np.array(ocfg['body_half_extents_m'])+.08
                            points=points[np.any(np.abs(points-np.array(pose))>half,axis=1)]
                    free_map=transform_points(free,sensor_world,self.scene['carla_world_origin_m'])
                    ray_origin=to_map_xyz([m.transform.location.x,m.transform.location.y,m.transform.location.z],self.scene['carla_world_origin_m']).tolist()
                    if kind=='lidar':
                        clear_range=float(sensor_cfg.get('clearing_range_m',max_range))
                        lidar_frame=prepare_lidar_frame(points,ray_origin,self.scene['resolution'],max_range,clear_range,self.scene['bounds'])
                        points,free_map,ray_stats=lidar_frame.hit_points,lidar_frame.free_endpoints,lidar_frame.stats
                        # Observed support is not the emitted scan span: no-return beams are absent.
                        ray_stats.update(return_azimuth_span_deg=raw_return_span,raw_return_count=raw_return_count,
                            scan_end_angle_rad=float(m.horizontal_angle))
                        far=np.zeros(len(free_map),dtype=bool);max_range=clear_range
                    else:ray_stats={}
                    fields=dict(scene_id=self.scene['scene_id'],scene_epoch=self.epoch,robot_id=rid,sensor_id=kind,frame_seq=int(m.frame),
                        capture_sim_time=float(m.timestamp),receive_wall_time=time.time(),calibration_id=sc['id']+'-'+kind,
                        origin=ray_origin,points=points.tolist(),max_range_m=max_range,compact=True,refresh_encoding='i16le_base64',
                        free_rays=[{'endpoint':p.tolist(),'source':('lidar_measured_beyond_range' if kind=='lidar' else ('carla_depth_far_plane' if f else 'registered_depth_measured_beyond_range'))} for p,f in zip(free_map,far)])
                    if kind=='lidar':
                        fields['protected_hit_points']=lidar_frame.protected_hit_points.tolist()
                        fields['mark_range_m']=float(sensor_cfg['effective_mapping_range_m'])
                        for ray in fields['free_rays']:ray['include_endpoint']=False
                    start=time.monotonic()
                    try:delta=self.mapper.request('integrate',**fields)
                    except MapError as exc:
                        self.event('map_frame_rejected',robot_id=rid,sensor=kind,frame=m.frame,reason=exc.code);continue
                    response_at=time.monotonic()
                    if kind=='lidar':
                        ray_stats.update(protected_hit_cells=delta.get('protected_hit_cells',0),
                            refreshed_protected_hits=delta.get('refreshed_protected_hits',0),
                            recovered_far_hit_cells=delta.get('recovered_far_hit_cells',0),
                            skipped_stale_cells=delta.get('skipped_stale_cells',0))
                    frame=FusionFrame(
                        robot_id=rid,sensor_id=kind,frame=int(m.frame),capture_sim_time=float(m.timestamp),
                        arrival_wall=arrival,selected_wall=now,request_started_wall=start,response_wall=response_at,
                        prepare_s=before_pose_lock-now,pose_lock_s=after_pose_lock-before_pose_lock,rays_s=start-after_pose_lock,
                        points=len(points),free_rays=len(free_map),ray_coverage=ray_stats,
                        calibration_id=sc['id'],sensor_to_world=sensor_world.tolist())
                    # The next latest-frame native request can overlap this apply.
                    # submit first drains the previous apply, so no old-frame queue
                    # forms and execution freshness changes only after application.
                    apply_pipeline.submit(self._apply_fusion,delta,frame,last_rgb)
        except Exception:
            self.map_error=traceback.format_exc();self.event('mapping_failed',error=self.map_error);self.stop.set()

    def _apply_fusion(self,delta,frame,last_rgb):
        """Apply one ordered delta; publish freshness only after the commit.

        Capture metadata is bound in FusionFrame before submitting to the sole
        apply worker. total_work_s remains selection-to-application wall time;
        apply_queue_s separates overlap/scheduling delay from actual cache work.
        """
        rid,kind=frame.robot_id,frame.sensor_id
        apply_started=time.monotonic()
        with self.map_lock:
            self.cache.apply(delta)
            transitions=self.display_index.apply(self.cache.cells,delta['changed_cells'])
            self.map_watermark=max(self.map_watermark,frame.capture_sim_time)
        applied_at=time.monotonic()
        self.map_timing.append(applied_at-frame.request_started_wall);self.map_timing=self.map_timing[-80:]
        completed_sim_time=self.now_sim()
        with self.state_lock:
            self.robots[rid]['sensor_fresh'][kind]={'sim_time':frame.capture_sim_time,'wall':applied_at,'frame':frame.frame}
            self.fusion_metrics[rid+'/'+kind]={
                'capture_sim_time':frame.capture_sim_time,'completed_sim_time':completed_sim_time,
                'capture_to_apply_sim_s':max(0.,completed_sim_time-frame.capture_sim_time),
                'arrival_to_apply_wall_s':applied_at-frame.arrival_wall,
                'map_request_s':frame.response_wall-frame.request_started_wall,
                'apply_queue_s':apply_started-frame.response_wall,'cache_apply_s':applied_at-apply_started,
                'points':frame.points,'free_rays':frame.free_rays,'ray_coverage':frame.ray_coverage,
                'occupancy_transitions':transitions,'transport':delta.get('_transport_timing',{}),
                'prepare_s':frame.prepare_s,'pose_lock_s':frame.pose_lock_s,'rays_s':frame.rays_s,
                'total_work_s':applied_at-frame.selected_wall}
        with self.sensor_lock:
            self.sensor_stats[(rid,kind)]['fused']+=1
        # Registered RGB evidence is latest-only and tied to this applied depth
        # frame. The worker is the only writer, including during final draining.
        if kind=='depth':
            with self.sensor_lock:rgb=self.rgb_history.get(rid,{}).get(frame.frame)
            if rgb and last_rgb.get(rid)!=frame.frame:
                folder=self.root/rid;folder.mkdir(exist_ok=True)
                rgba=np.frombuffer(rgb.raw_data,dtype=np.uint8).reshape(rgb.height,rgb.width,4)
                try:
                    import cv2
                    cv2.imwrite(str(folder/'latest_rgb.tmp.jpg'),rgba[:,:,:3],[cv2.IMWRITE_JPEG_QUALITY,80])
                    (folder/'latest_rgb.tmp.jpg').replace(folder/'latest_rgb.jpg');last_rgb[rid]=frame.frame
                    atomic_json(folder/'latest_rgb.json',{'frame':frame.frame,'capture_sim_time':frame.capture_sim_time,
                        'calibration_id':frame.calibration_id,'sensor_to_world':frame.sensor_to_world})
                except ImportError:pass

    @staticmethod
    def remove_body_hits(local,extrinsic,cfg):
        if len(local)==0:return local
        body=local@extrinsic[:3,:3].T+extrinsic[:3,3]
        half=np.array(cfg['body_half_extents_m'])+.30;offset=np.array(cfg['body_offset_m'])
        return local[np.any(np.abs(body-offset)>half,axis=1)]

    def now_sim(self):
        return max(self.sim_time,self.capture_sim_time)

    def dynamic_boxes(self,rid,sim_time=None):
        reference=self.now_sim() if sim_time is None else sim_time
        with self.state_lock:
            return [{'id':other,'center_xyz':s['position'],'half_extents_xyz':s['instance']['robot_config']['body_half_extents_m'],
                     'valid_until_sim_time':reference+1.0} for other,s in self.robots.items() if other!=rid]

    @staticmethod
    def person_document_at(doc,reference):
        # Registry publication time can advance between choosing a planner
        # time and reading its atomic document. A newer header alone does not
        # make older measurements future data. Rebase only that watermark;
        # build_hazards still validates every observation and its original TTL.
        if not isinstance(doc,dict):return doc
        published=doc.get('reference_sim_time')
        if type(published) not in (int,float) or not math.isfinite(published) or abs(published)>1e7:
            return doc  # Preserve malformed fields for the normal strict validator.
        if published<=reference:return doc
        records=doc.get('people')
        if not isinstance(records,list):return doc
        for record in records:
            if not isinstance(record,dict):return doc
            for stamp in (record.get('capture_sim_time'),record.get('position_reference_sim_time',published)):
                if type(stamp) not in (int,float) or not math.isfinite(stamp) or stamp>reference:
                    return doc  # Actual future measurement/projection is never accepted.
        return dict(doc,reference_sim_time=reference)

    def person_hazard_snapshot(self,robot_ids,sim_time=None):
        """One capture-valid document and one time for execution/display clients.

        Native planner results retain their own historical people_hazards. They
        must not pace live people display or extend observation expiration.
        """
        robot_ids=tuple(robot_ids)
        enabled=self.scene.get('person_prediction_enabled',False)
        doc=None;read_error=None
        if enabled:
            try:doc=json.loads((self.root/'shared/people.json').read_text())
            except (ValueError,KeyError,TypeError,OSError) as exc:read_error=exc
        # Sample the runtime clock after I/O, not before: callbacks may have
        # advanced while the new document was read. Explicit planner times stay
        # exact, so the native request and its people layer have one reference.
        reference=self.now_sim() if sim_time is None else sim_time
        if not enabled:return reference,{rid:[] for rid in robot_ids}
        selected=self.person_document_at(doc,reference)
        hazards={};errors=[]
        for rid in robot_ids:
            cfg=self.robots[rid]['instance']['robot_config']
            try:
                if read_error is not None:raise read_error
                hazards[rid]=build_hazards(selected,cfg,reference,self.epoch,self.scene['scene_id'])
                self.people_documents[rid]=selected
            except (ValueError,KeyError,TypeError,OSError) as exc:
                errors.append(type(exc).__name__+': '+str(exc))
                # A malformed/newer observation is not authoritative absence.
                # Reuse only validated evidence within its original expiry.
                try:hazards[rid]=build_hazards(self.people_documents.get(rid),cfg,reference,self.epoch,self.scene['scene_id'])
                except (ValueError,KeyError,TypeError):hazards[rid]=[]
        error=errors[0] if errors else None
        if error is not None and error!=self.people_error:self.event('person_hazards_deferred',reason=error)
        self.people_error=error
        return reference,hazards

    def person_hazards(self,rid,sim_time=None):
        return self.person_hazard_snapshot((rid,),sim_time)[1][rid]

    def planner_loop(self,rid):
        s=self.robots[rid];cfg=s['instance']['robot_config'];dim=cfg['dimension'];worker=None
        try:
            while not self.stop.is_set():
                with self.map_lock:ready=self.cache.revision>=6
                if ready and s['sensor_fresh'].get('lidar'):break
                self.stop.wait(.1)
            if self.stop.is_set():return
            binary=ROOT/'planning/target/release/air-ground-smart3d-worker'
            worker=JsonProcess([str(binary)],self.root/(rid+'_planner.stderr'),timeout=40);s['worker']=worker
            map_revision=0;goal_revision=0;iteration=0;session_generation=0

            def recover_invalidated_session():
                # The native worker explicitly discarded its session after a
                # panic. Keep the shared mapper/robots alive, revoke the old
                # path, and initialize only this planner from current evidence.
                nonlocal map_revision,goal_revision,session_generation
                session_generation+=1;map_revision=0;goal_revision=0
                with self.state_lock:
                    s['planner']={};s['speed']=0;s['last_plan_wall']=0.
                    s['status']='planner_reinitializing';s['planner_recoveries']=session_generation
                self.event('planner_session_invalidated',robot_id=rid,
                    generation=session_generation,shared_map_preserved=True)
                self.stop.wait(min(5.,float(session_generation)))

            while not self.stop.is_set():
                with self.state_lock:
                    pos=list(s['position']);goal=list(s['instance']['route'][s['route_index']]);requested_revision=s['goal_revision'];sim=self.now_sim()
                op='init' if goal_revision==0 else ('set_goal' if goal_revision!=requested_revision else 'update')
                session_id=self.epoch+'-'+rid+(('-recovery'+str(session_generation)) if session_generation else '')
                with self.state_lock:s['planner_session_id']=session_id
                request={'op':op,'robot_id':rid,'session_id':session_id,'goal_revision':requested_revision,
                         'current':pos[:dim],'sim_time':sim,'dynamic_revision':iteration,'dynamic_boxes':self.dynamic_boxes(rid,sim),'people_hazards':self.person_hazards(rid,sim),'include_tree':True}
                if op=='init':
                    snapshot=self.mapper.request('snapshot')
                    bounds=self.scene['flight_bounds'] if dim==3 else self.scene['bounds']
                    request.update(protocol='air_ground_smart3d.v1',dimension=dim,start=pos[:dim],goal=goal[:dim],ranges=bounds[:dim],
                        profile={'profile_id':cfg['profile_id'],'body_half_extents_m':cfg['body_half_extents_m'],'body_offset_m':cfg['body_offset_m'],'planar_z':cfg.get('planar_z',0),'cluster_inflation_m':float(cfg.get('cluster_inflation_m') or 0)},
                        map_snapshot=snapshot,initial_iterations=self.team['planning']['initial_iterations_'+str(dim)+'d'],repair_iterations=self.team['planning']['repair_iterations'])
                    del snapshot  # request owns it only until initialization completes
                else:
                    try:request['map_delta']=self.mapper.request('changes',since_revision=map_revision,compact=True,refresh_semantics='presence_only')
                    except MapError as exc:
                        if 'snapshot' not in exc.code and 'history' not in exc.code:raise
                        request['map_snapshot']=self.mapper.request('snapshot')
                    if op=='set_goal':request['goal']=goal[:dim]
                start=time.monotonic();result=worker.call(request)
                if result.get('ok') is False:
                    reason=result.get('reason','unknown_rejection')
                    with self.state_lock:
                        if reason in ('goal_blocked_or_out_of_bounds','start_blocked_or_out_of_bounds') and s['planner']:
                            s['planner']=dict(s['planner'],path_xyz=None,path_status='awaiting_goal_update')
                        else:s['planner']={}
                        s['status']='planner_hold_'+reason;s['speed']=0
                    self.event('planner_request_rejected',robot_id=rid,op=op,reason=reason)
                    if reason=='native_panic_session_invalidated':
                        recover_invalidated_session();continue
                    if reason not in ('goal_blocked_or_out_of_bounds','start_blocked_or_out_of_bounds'):
                        # Malformed data is a contract failure, not a reason to
                        # flood the shared mapper with repeated full snapshots.
                        raise RuntimeError(reason)
                    # Rejected set_goal is transactional. Keep accepted revision and
                    # apply new map to the existing tree before trying the goal again.
                    if op!='init':
                        with self.state_lock:pos=list(s['position']);sim=self.now_sim()
                        recovery={'op':'update','robot_id':rid,'session_id':session_id,'goal_revision':goal_revision,
                                  'current':pos[:dim],
                                  'sim_time':sim,'dynamic_revision':iteration+1,'dynamic_boxes':self.dynamic_boxes(rid,sim),'people_hazards':self.person_hazards(rid,sim),'include_tree':True}
                        # Rejection leaves the worker's accepted map unchanged.
                        # Reuse that exact unapplied delta (or history-gap snapshot)
                        # rather than serialize the full shared map on each retry.
                        map_field='map_delta' if 'map_delta' in request else 'map_snapshot'
                        recovery[map_field]=request[map_field]
                        recovered=worker.call(recovery)
                        if recovered.get('ok') is not True:
                            if recovered.get('reason')=='native_panic_session_invalidated':
                                recover_invalidated_session();continue
                            raise RuntimeError('planner_recovery_rejected: '+recovered.get('reason','unknown_rejection'))
                        if self.scene.get('person_prediction_enabled',False) and not all(k in recovered for k in ('people_revision','people_active_hazards','people_hazards')):
                            raise RuntimeError('worker_missing_people_hazard_capability')
                        map_revision=recovered['map']['revision']
                        # Native update has revalidated the existing forest on
                        # the new map. Show that real tree while the requested
                        # goal is blocked, but never execute the previous goal.
                        recovered.update(path_xyz=None,path_status='awaiting_goal_update',
                            goal_revision=goal_revision,requested_goal_revision=requested_revision,
                            received_wall=time.time())
                        with self.state_lock:
                            s['planner']=recovered;s['last_plan_wall']=time.monotonic();s['speed']=0
                        request=None;recovery=None
                        iteration+=2
                    self.stop.wait(2. if op=='init' else .5)
                    continue
                request=None  # release initialization/recovery map payloads promptly
                if self.scene.get('person_prediction_enabled',False) and not all(k in result for k in ('people_revision','people_active_hazards','people_hazards')):
                    raise RuntimeError('worker_missing_people_hazard_capability')
                result['latency_s']=time.monotonic()-start;result['received_wall']=time.time();result['goal_revision']=requested_revision
                with self.state_lock:
                    if requested_revision==s['goal_revision']:
                        s['planner']=result;s['last_plan_wall']=time.monotonic();s.pop('planner_error',None)
                map_revision=result['map']['revision'];goal_revision=requested_revision;iteration+=1
                if op!='update' or iteration%10==0:self.event('planner_result',robot_id=rid,op=op,map_revision=map_revision,
                    status=result.get('path_status'),path_points=len(result.get('path_xyz') or []),tree_instance_id=result.get('tree_instance_id'),latency_s=result['latency_s'])
                self.stop.wait(max(.02,self.team['planning']['period_s']-result['latency_s']))
        except Exception:
            with self.state_lock:s['planner_error']=traceback.format_exc();s['status']='planner_error';s['planner']={};s['speed']=0
            self.event('planner_failed',robot_id=rid,error=s['planner_error'])
        finally:
            if worker:worker.close()

    def recover_overlap(self,s,dt):
        cfg=s['instance']['robot_config'];pos=s['position'];fresh=s['sensor_fresh'].get('lidar',{})
        if self.sim_time-fresh.get('sim_time',-999)>cfg['execution_guard']['sensor_max_age_s']:
            s['status']='sensor_stale';return None
        bounds={'min':[b[0] for b in self.scene['bounds']], 'max':[b[1] for b in self.scene['bounds']]}
        if cfg['dimension']==3:
            # Guard bounds contain the whole body; flight bounds contain its reference center.
            for i,(lo,hi) in enumerate(self.scene['flight_bounds']):
                bounds['min'][i]=max(bounds['min'][i],lo+cfg['body_offset_m'][i]-cfg['body_half_extents_m'][i])
                bounds['max'][i]=min(bounds['max'][i],hi+cfg['body_offset_m'][i]+cfg['body_half_extents_m'][i])
        with self.map_lock:
            target=s.get('recovery_target')
            if target is None:
                target=find_recovery_target(self.cache.cells,self.scene['resolution'],pos,cfg['body_half_extents_m'],cfg['body_offset_m'],
                    self.sim_time,dimension=cfg['dimension'],max_distance_m=.6,max_free_age=cfg['execution_guard']['free_evidence_max_age_s'],bounds=bounds)
                if target is None:s['status']='current_blocked_no_clear_exit';return None
                s['recovery_target']=target
                self.event('bounded_overlap_exit',robot_id=s['instance']['robot_id'],start_xyz=pos,target_xyz=target,map_changed=False,tree_reset=False)
            delta=np.array(target)-np.array(pos);distance=float(np.linalg.norm(delta))
            next_pos=(np.array(pos)+delta*min(distance,.25*dt)/max(distance,1e-12)).tolist()
            check=check_recovery_segment(self.cache.cells,self.scene['resolution'],pos,next_pos,cfg['body_half_extents_m'],cfg['body_offset_m'],
                self.sim_time,dimension=cfg['dimension'],max_free_age=cfg['execution_guard']['free_evidence_max_age_s'],bounds=bounds,require_clear_end=False)
        if not check['ok']:s.pop('recovery_target',None);s['status']='overlap_exit_blocked';return None
        from execution.guard import _segment_box
        for other in self.dynamic_boxes(s['instance']['robot_id']):
            half=np.array(other['half_extents_xyz'])+np.array(cfg['body_half_extents_m']);center=np.array(other['center_xyz'])
            if _segment_box(pos,next_pos,center-half,center+half):s['status']='overlap_exit_partner_hold';return None
        hazard=check_hazard_segment(pos,next_pos,self.person_hazards(s['instance']['robot_id']))
        if hazard:s['status']='overlap_exit_person_hold';return None
        if distance<.015:s.pop('recovery_target',None)
        s['speed']=0;s['status']='exiting_newly_observed_overlap';s['guard']=check
        c=to_carla_xyz(next_pos,self.scene['carla_world_origin_m'])
        return carla.command.ApplyTransform(s['actor'].id,carla.Transform(carla.Location(*c),carla.Rotation(yaw=-math.degrees(s['yaw']))))

    def control(self,s,dt):
        cfg=s['instance']['robot_config'];pos=s['position'];route=s['instance']['route'];goal=route[s['route_index']]
        if math.dist(pos,goal)<cfg['goal_tolerance_m']:
            s['completed_goals']+=1;s['route_index']=(s['route_index']+1)%len(route);s['goal_revision']+=1;s['speed']=0;s['status']='goal_reached'
            if s['planner']:s['planner']=dict(s['planner'],path_xyz=None,path_status='awaiting_goal_update')
            self.event('goal_reached',robot_id=s['instance']['robot_id'],goal_xyz=goal,next_goal=route[s['route_index']]);return None
        plan=s['planner'];path=plan.get('path_xyz') or []
        if plan and time.monotonic()-s['last_plan_wall']>3:
            s['status']='planner_stale';s['speed']=0;return None
        if plan:
            with self.map_lock:
                identity=plan.get('map',{})
                if (plan.get('robot_id')!=s['instance']['robot_id'] or plan.get('session_id')!=s.get('planner_session_id',self.epoch+'-'+s['instance']['robot_id']) or
                    any(identity.get(k)!=v for k,v in self.cache.identity.items()) or identity.get('revision',-1)>self.cache.revision):
                    s['status']='waiting_map_revision';s['speed']=0;return None
        if plan.get('goal_revision')!=s['goal_revision'] or len(path)<2:
            s['speed']=0
            if plan.get('goal_revision')==s['goal_revision'] and (plan.get('path_status')=='current_blocked_or_out_of_bounds' or s.get('recovery_target') is not None):
                return self.recover_overlap(s,dt)
            s['status']='waiting_path';return None
        s.pop('recovery_target',None)
        if time.monotonic()-s['last_plan_wall']>3:s['status']='planner_stale';s['speed']=0;return None
        fresh=s['sensor_fresh'].get('lidar',{})
        if self.sim_time-fresh.get('sim_time',-999)>cfg['execution_guard']['sensor_max_age_s']:
            s['status']='sensor_stale';s['speed']=0;return None
        target=lookahead(pos,path,.7)
        if cfg['dimension']==2:target[2]=cfg['planar_z']
        next_pos,next_yaw,speed=bounded_step(pos,s['yaw'],target,s['speed'],cfg['max_speed_mps'],cfg['max_acceleration_mps2'],cfg['max_yaw_rate_rps'],dt)
        # Only imminent travel is released, the full unknown-region tree is a candidate.
        direction=np.array(target)-np.array(pos);length=float(np.linalg.norm(direction))
        near=(np.array(pos)+direction*min(length,cfg['execution_guard']['near_path_m'])/max(length,1e-12)).tolist()
        with self.map_lock:
            identity=plan.get('map',{})
            if (plan.get('robot_id')!=s['instance']['robot_id'] or plan.get('session_id')!=s.get('planner_session_id',self.epoch+'-'+s['instance']['robot_id']) or
                any(identity.get(k)!=v for k,v in self.cache.identity.items()) or identity.get('revision',-1)>self.cache.revision):
                s['status']='waiting_map_revision';s['speed']=0;return None
            check=check_path_segment(self.cache.cells,self.scene['resolution'],pos,near,cfg['body_half_extents_m'],cfg['body_offset_m'],
                self.sim_time,cfg['execution_guard']['free_evidence_max_age_s'],{'min':[b[0] for b in self.scene['bounds']], 'max':[b[1] for b in self.scene['bounds']]})
        if not check['ok'] and check.get('blocked_key') is not None:
            with self.map_lock:cell=self.cache.cells.get(tuple(check['blocked_key']),{})
            check['last_observed_sim_time']=cell.get('last_observed_sim_time')
            check['observed_state']=cell.get('state','unknown')
        if check['ok']:
            from execution.guard import _segment_box
            for other in self.dynamic_boxes(s['instance']['robot_id']):
                half=np.array(other['half_extents_xyz'])+np.array(cfg['body_half_extents_m']);center=np.array(other['center_xyz'])
                if _segment_box(pos,near,center-half,center+half):
                    check={'ok':False,'reason':'live_partner','blocked_key':None};break
        if check['ok']:
            hazard=check_hazard_segment(pos,near,self.person_hazards(s['instance']['robot_id']))
            if hazard:check={'ok':False,'reason':'person_ohz','person_id':hazard['id'],'blocked_key':None}
        if not check['ok']:
            next_pos=pos;s['speed']=0;s['status']='guard_'+check['reason'];s['guard']=check
        else:s['speed']=speed;s['status']='moving' if speed>.01 else 'turning';s['guard']=check
        c=to_carla_xyz(next_pos,self.scene['carla_world_origin_m'])
        return carla.command.ApplyTransform(s['actor'].id,carla.Transform(carla.Location(*c),carla.Rotation(yaw=-math.degrees(next_yaw))))

    def publish(self):
        started=time.monotonic()
        people_reference,current_people=self.person_hazard_snapshot(tuple(self.robots))
        with self.state_lock:
            robots=[]
            for rid,s in self.robots.items():
                cfg=s['instance']['robot_config']
                robots.append({'robot_id':rid,'actor_id':s['actor'].id,'kind':cfg['mobility'],'dimension':cfg['dimension'],
                    'position_xyz':s['position'],'yaw':s['yaw'],'body_half_extents_m':cfg['body_half_extents_m'],'body_offset_m':cfg['body_offset_m'],
                    'goal_xyz':s['instance']['route'][s['route_index']],
                    'people_hazards':current_people[rid],'people_hazard_reference_sim_time':people_reference,
                    **{k:s.get(k) for k in ['status','planner','planner_error','planner_recoveries','distance_m','vertical_distance_m','actual_speed','completed_goals','goal_revision','guard','last_control_error','motion_block_count','control_latency_s']},'sensor_fresh':{k:dict(v) for k,v in s['sensor_fresh'].items()}})
        # Display snapshots are immutable and separate from executable in-memory
        # plans. Never hold the control-state lock during tree encoding or I/O.
        for robot in robots:
            robot['planner']=self.tree_snapshots.compact(robot['robot_id'],robot['planner'])
        with self.map_lock:
            free_xy,blocked_xy,occupied=self.display_index.snapshot()
            revision=self.cache.revision;total=len(self.cache.cells);watermark=self.map_watermark
        state={'schema':'air_ground.state/v1','scene_id':self.scene['scene_id'],'epoch':self.epoch,'wall_time':time.time(),
            'sim_time':people_reference,'people_hazard_reference_sim_time':people_reference,'world_id':self.world_id,'running':not self.stop.is_set(),'robots':robots,
            'human_count':len(self.people.actors) if self.people else 0,'map_summary':{'revision':revision,'observed_cells':total,'occupied_cells':len(occupied),
                'watermark':watermark,'fusion_latency_mean_s':float(np.mean(self.map_timing)) if self.map_timing else None},
            'sensor_stats':{rid+'/'+kind:dict(stats) for (rid,kind),stats in list(self.sensor_stats.items())},'fusion_metrics':dict(self.fusion_metrics),'publish_latency_mean_s':float(np.mean(self.publish_timing)) if self.publish_timing else None,'map_error':self.map_error,'fatal':self.fatal,
            'motion_rpc_latency_s':self.motion_rpc_latency_s,'gc_metrics':dict(self.gc_metrics),'json_codec':json.backend_info(),'person_prediction_enabled':bool(self.scene.get('person_prediction_enabled',False)),'people_error':self.people_error,'planner_core':'LINKS Lab SMART<f32,2> / SMART<f32,3>'}
        atomic_json(self.root/'state.json',state)
        atomic_json(self.root/'shared/display.json',{'scene_epoch':self.epoch,'revision':revision,'resolution':self.scene['resolution'],
            'occupied':occupied,'free_xy':free_xy,'blocked_xy':blocked_xy,'projection_semantics':'uninflated_body_height_band_not_execution_clearance','ground_z':self.scene['ground_z_m'],'sim_time':self.now_sim(),'map_capture_watermark':watermark,'published_wall_time':time.time()})
        self.publish_timing.append(time.monotonic()-started);self.publish_timing=self.publish_timing[-80:]

    def publication_loop(self):
        # Atomic latest snapshots are display/telemetry only. Never execute them
        # inside the control tick, and never queue a backlog of old snapshots.
        period=1./float(self.team.get('observability',{}).get('snapshot_hz',5.))
        while not self.stop.is_set():
            started=time.monotonic()
            try:self.publish()
            except Exception as exc:self.event('publication_failed',error=str(exc))
            self.stop.wait(max(.01,period-(time.monotonic()-started)))

    def run(self):
        signal.signal(signal.SIGTERM,lambda *_:self.stop.set());signal.signal(signal.SIGINT,lambda *_:self.stop.set())
        try:
            self.spawn()
            self.motion=MotionClient(self.scene['carla_host'],self.scene['carla_port'],self.world_id,self.epoch,[s['actor'].id for s in self.robots.values()],self.root/'motion_rpc.stderr')
            for fn,args in [(self.mapping_loop,()),(self.publication_loop,())]+[(self.planner_loop,(rid,)) for rid in self.robots]:
                t=threading.Thread(target=fn,args=args,daemon=True);t.start();self.threads.append(t)
            last_sim=None;last_people=0
            while not self.stop.is_set():
                snapshot=self.world.wait_for_tick(2)
                if snapshot is None:raise RuntimeError('simulator tick timeout')
                if self.world.id!=self.world_id:raise RuntimeError('scene epoch changed')
                sim=snapshot.timestamp.elapsed_seconds;dt=min(.075,max(.001,sim-last_sim)) if last_sim is not None else .03;last_sim=sim
                self.sim_time=sim;history={}
                with self.state_lock:
                    for rid,s in self.robots.items():
                        a=snapshot.find(s['actor'].id)
                        if a is None:raise RuntimeError('owned actor missing: '+rid)
                        tr=a.get_transform();p=to_map_xyz([tr.location.x,tr.location.y,tr.location.z],self.scene['carla_world_origin_m']).tolist()
                        delta=math.dist(p,s['position']);s['distance_m']+=delta;s['vertical_distance_m']+=abs(p[2]-s['position'][2]);s['actual_speed']=delta/dt
                        s['position']=p;s['yaw']=-math.radians(tr.rotation.yaw);history[rid]=p
                    commands=[];states=[]
                    for s in self.robots.values():
                        control_started=time.monotonic();command=self.control(s,dt);s['control_latency_s']=time.monotonic()-control_started
                        if command is not None:commands.append(command);states.append(s)
                if commands:
                    motion_started=time.monotonic()
                    responses=self.motion.apply_batch_sync(commands,False)
                    self.motion_rpc_latency_s=time.monotonic()-motion_started
                    for s,result in zip(states,responses):
                        if result.error:
                            s.pop('recovery_target',None);s['speed']=0;s['status']='ue_blocked';s['last_control_error']=result.error;s['motion_block_count']+=1
                            if s['motion_block_count']%30==1:self.event('motion_blocked',robot_id=s['instance']['robot_id'],reason=result.error,position=s['position'])
                        else:s['last_control_error']=None
                if sim-last_people>.15:self.people.update(sim-last_people if last_people else .15);last_people=sim
        except Exception:
            self.fatal=traceback.format_exc();self.event('runtime_failed',error=self.fatal);raise
        finally:
            self.stop.set()
            gc.callbacks.remove(self.record_gc)
            for t in self.threads:t.join(timeout=45)
            try:self.mapper.request('save',path=str(self.root/'shared/map.ot'))
            except Exception as exc:self.event('snapshot_failed',error=str(exc))
            try:self.publish()
            except Exception:pass
            if self.motion is not None:self.motion.close()
            if self.tick_callback_id is not None:
                try:self.world.remove_on_tick(self.tick_callback_id)
                except Exception:pass
            if self.people:
                try:self.people.close()
                except Exception:pass
            for s in self.robots.values():
                for _,sensor,_ in s['sensors']:
                    try:sensor.stop();sensor.destroy()
                    except Exception:pass
                try:s['actor'].destroy()
                except Exception:pass
            self.mapper.close();self.events.close()


if __name__=='__main__':
    parser=argparse.ArgumentParser();parser.add_argument('--team',default='go2_fighter');parser.add_argument('--resume-map');parser.add_argument('--resume-state');args=parser.parse_args()
    Runtime(load_team(args.team),resume_map=args.resume_map,resume_state=args.resume_state).run()
