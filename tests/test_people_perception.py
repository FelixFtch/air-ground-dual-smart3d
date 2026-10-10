"""CPU-only registered projection, cross-camera association and service contracts."""
import copy
import hashlib
import json
from pathlib import Path
import tempfile
from types import SimpleNamespace
import unittest
from unittest.mock import patch

import numpy as np

from perception.people_projection import (PeopleProjector, local_key, VELOCITY_WINDOW_SECONDS,
                                          VELOCITY_MAX_GAP_SECONDS, VELOCITY_HISTORY_SAMPLES)
from perception.person_registry import PersonRegistry
from perception.sam3_backend import VideoModel
from perception.service import FrameSequence, LatestFrameScheduler, PeopleService, overlay


def metadata(robot='go2', stamp=10., frame=1):
    return dict(schema='air_ground.registered_rgbd/v1', scene_id='scene',scene_epoch='epoch',
        robot_id=robot,camera_id='front_rgbd',calibration_id='cal', frame_seq=frame,
        capture_sim_time=stamp,frame_token=str(frame)+robot,width=40,height=40,
        rgb_sha256='a'*64,receive_wall_time=1000.,camera_to_map=np.eye(4).tolist(),
        body_to_map=np.eye(4).tolist(),K=np.eye(3).tolist())


def frame(meta=None):
    meta=metadata() if meta is None else meta
    yy,xx=np.mgrid[-.6:.6:40j,-.2:.2:40j]
    xyz=np.stack((xx,yy,np.ones_like(xx)*3),axis=-1).astype(np.float32)
    return dict(rgb=np.full((40,40,3),100,dtype=np.uint8),xyz_optical=xyz,meta=meta)


def result(session='session', native=1):
    return dict(tracking_session_id=session,tracks=[dict(id=native,box=[0.,0.,39.,39.],score=.9)],
                masks=np.ones((1,40,40),dtype=bool),inference_ms=0.,session_frame_index=0)


def observation(robot='go2', native=1, xyz=(0,0,1), stamp=10., valid=False, velocity=(0,0,0)):
    meta=metadata(robot,stamp)
    return dict(scene_id='scene',scene_epoch='epoch',robot_id=robot,camera_id='front_rgbd',
        native_track_id=native,track_id=native,source_key=local_key(meta,'session',native),
        center_xyz=list(xyz),half_extents_xyz=[.2,.2,.5],capture_sim_time=stamp,
        velocity_xyz=list(velocity),velocity_xy_valid=valid,velocity_z_valid=False,
        quality=dict(level='good',extent_kind='visible_surface_quantiles',
            human_height_prior_applied=False,calibration_status='uncalibrated'))


class ProjectionTests(unittest.TestCase):
    def test_four_measurement_gate_and_capture_time_speed(self):
        projector=PeopleProjector()
        for i in range(4):
            capture=frame(metadata(stamp=10+i*.2,frame=i))
            capture['xyz_optical'][:,:,0]+=i*.2
            obs=projector.project(capture['meta'],result(),capture['xyz_optical'])['tracks'][0]
            self.assertEqual(obs['velocity_xy_valid'], i==3)
        np.testing.assert_allclose(obs['velocity_xyz'],[1,0,0],atol=1e-5)
        self.assertFalse(obs['velocity_z_valid'])
        self.assertEqual(obs['observation_samples'],4)
        self.assertFalse(obs['quality']['human_height_prior_applied'])
        self.assertLess(obs['half_extents_xyz'][2],1e-6)

    def test_one_hz_measured_speed_accuracy_and_bounded_fit_history(self):
        projector=PeopleProjector()
        expected=np.array([.35,-.12])
        for i in range(12):
            capture=frame(metadata(stamp=10.+i,frame=i))
            capture['xyz_optical'][:,:,:2]+=expected*i
            obs=projector.project(capture['meta'],result(),capture['xyz_optical'])['tracks'][0]
            self.assertEqual(obs['velocity_xy_valid'],i>=3)
            if i>=3:
                np.testing.assert_allclose(obs['velocity_xyz'],[.35,-.12,0.],atol=1e-5)
            retained=next(iter(projector.history.values()))
            self.assertLessEqual(len(retained),VELOCITY_HISTORY_SAMPLES)
            self.assertLessEqual(retained[-1][0]-retained[0][0],VELOCITY_WINDOW_SECONDS)
        self.assertFalse(obs['velocity_z_valid'])
        self.assertEqual(obs['velocity_fit_window_seconds'],4.5)
        self.assertEqual(obs['velocity_max_capture_gap_seconds'],1.5)

    def test_one_hz_gap_reset_keeps_existing_threshold_and_requires_four_again(self):
        projector=PeopleProjector()
        for i in range(4):
            answer=projector._velocity('native',10.+i,[.4*i,0.])
        self.assertTrue(answer[1])
        stamp=13.+VELOCITY_MAX_GAP_SECONDS+.001
        answer=projector._velocity('native',stamp,[.4*(stamp-10.),0.])
        self.assertEqual(answer[1:],(False,'capture_gap_or_position_jump',1))
        for i in range(1,4):
            answer=projector._velocity('native',stamp+i,[.4*(stamp+i-10.),0.])
            self.assertEqual(answer[1],i==3)
        np.testing.assert_allclose(answer[0],[.4,0.],atol=1e-9)

    def test_one_hz_stale_capture_cannot_publish_or_seed_velocity(self):
        projector=PeopleProjector()
        capture=frame()
        for i in range(4):
            meta=metadata(stamp=10.+i,frame=i)
            obs=projector.project(meta,result(),capture['xyz_optical'])['tracks'][0]
        self.assertTrue(obs['velocity_xy_valid'])
        expired=projector.project(metadata(stamp=14.,frame=4),result(),capture['xyz_optical'],stale=True)
        self.assertEqual(expired['tracks'],[])
        self.assertEqual(expired['error'],'source_expired')
        self.assertFalse(projector.history)
        fresh=projector.project(metadata(stamp=15.,frame=5),result(),capture['xyz_optical'])['tracks'][0]
        self.assertFalse(fresh['velocity_xy_valid'])
        self.assertEqual(fresh['observation_samples'],1)

    def test_camera_motion_and_pitch_do_not_create_person_velocity(self):
        projector=PeopleProjector()
        world=frame()['xyz_optical'].astype(float)
        for i in range(5):
            meta=metadata(stamp=10+i*.2,frame=i)
            angle=i*.03
            c,s=np.cos(angle),np.sin(angle)
            transform=np.array([[c,0,s,i*.1],[0,1,0,0],[-s,0,c,0],[0,0,0,1]])
            local=(world-transform[:3,3])@transform[:3,:3]
            meta['camera_to_map']=transform.tolist()
            obs=projector.project(meta,result(),local.astype(np.float32))['tracks'][0]
        self.assertTrue(obs['velocity_xy_valid'])
        np.testing.assert_allclose(obs['velocity_xyz'],[0,0,0],atol=1e-5)

    def test_reject_depth_dimensions_and_clear_stale_or_changed_calibration(self):
        projector=PeopleProjector();capture=frame()
        with self.assertRaisesRegex(ValueError,'dimensions'):
            projector.project(capture['meta'],result(),capture['xyz_optical'][:-1])
        bad=capture['xyz_optical'].copy();bad[:]=np.nan
        self.assertEqual(projector.project(capture['meta'],result(),bad)['rejected'][0]['reason'],'insufficient_mask_depth')
        for i in range(4):
            meta=metadata(stamp=10+i*.2,frame=i)
            obs=projector.project(meta,result(),capture['xyz_optical'])['tracks'][0]
        self.assertTrue(obs['velocity_xy_valid'])
        meta=metadata(stamp=11,frame=5);meta['calibration_id']='new'
        self.assertFalse(projector.project(meta,result(),capture['xyz_optical'])['tracks'][0]['velocity_xy_valid'])
        self.assertFalse(projector.project(meta,result(),capture['xyz_optical'],stale=True)['tracks'])
        self.assertFalse(projector.history)

    def test_scoped_native_ids_and_unknown_vertical_speed(self):
        a,b=PeopleProjector(),PeopleProjector()
        for i in range(4):
            f=frame(metadata('go2',10+i*.2,i));f['xyz_optical'][:,:,2]+=i*.1
            first=a.project(f['meta'],result(),f['xyz_optical'])['tracks'][0]
            f['meta']['robot_id']='uav'
            second=b.project(f['meta'],result(),f['xyz_optical'])['tracks'][0]
        self.assertNotEqual(first['source_key'],second['source_key'])
        self.assertFalse(first['velocity_z_valid']);self.assertEqual(first['velocity_xyz'][2],0)


class RegistryTests(unittest.TestCase):
    def setUp(self):self.registry=PersonRegistry('scene','epoch')
    def add(self,robot,values,stamp=10):
        return self.registry.update(('epoch',robot,'front_rgbd'),values,stamp)

    def test_same_native_ids_are_scoped_and_unambiguous_views_merge(self):
        a=observation('go2');b=observation('uav',xyz=(.25,0,1))
        first=self.add('go2',[a]);second=self.add('uav',[b])
        self.assertEqual(first[a['source_key']],second[b['source_key']])
        people=self.registry.snapshot(10,1000)['people']
        self.assertEqual(len(people),1);self.assertEqual(len(people[0]['source_keys']),2)
        self.assertFalse(people[0]['velocity_xy_valid'])
        self.assertFalse(people[0]['velocity_z_valid'])

    def test_ambiguous_close_people_are_never_forced_to_nearest(self):
        self.add('go2',[observation('go2',1,(-.25,0,1)),observation('go2',2,(.25,0,1))])
        self.add('uav',[observation('uav',1,(0,0,1))])
        people=self.registry.snapshot(10,1000)['people']
        self.assertEqual(len(people),3)
        self.assertTrue(any(p['quality']['association']=='ambiguous_new_identity' for p in people))

    def test_two_new_tracks_cannot_both_claim_one_existing_person(self):
        self.add('go2',[observation()])
        self.add('uav',[observation('uav',1,(-.2,0,1)),observation('uav',2,(.2,0,1))])
        self.assertEqual(len(self.registry.snapshot(10,1000)['people']),3)

    def test_capture_expiry_and_extrapolation_are_bounded(self):
        self.add('go2',[observation(valid=True,velocity=(1,0,0))])
        item=self.registry.snapshot(11.4,1000)['people'][0]
        self.assertEqual(item['center_xyz'],[1.,0.,1.])
        self.assertEqual(item['position_reference_sim_time'],11.)
        self.assertEqual(item['valid_until_sim_time'],11.5)
        self.assertEqual(self.registry.snapshot(11.5,1000)['people'],[])

    def test_fused_geometry_cannot_extend_older_contributor_ttl(self):
        self.add('go2',[observation(stamp=10.)],10.)
        self.add('uav',[observation('uav',stamp=10.5)],10.5)
        self.assertEqual(self.registry.snapshot(10.5,1000)['people'][0]['valid_until_sim_time'],11.5)
        self.assertEqual(self.registry.snapshot(11.6,1000)['people'][0]['valid_until_sim_time'],12.)

    def test_view_surface_offsets_do_not_generate_speed(self):
        for i in range(4):
            stamp=10+i*.2
            self.add('go2',[observation('go2',xyz=(-.2,0,1),stamp=stamp,valid=True)],stamp)
            self.add('uav',[observation('uav',xyz=(.2,0,1),stamp=stamp,valid=True)],stamp)
        people=self.registry.snapshot(stamp,1000)['people']
        self.assertEqual(len(people),1)
        self.assertEqual(people[0]['velocity_xyz'],[0.,0.,0.])
        self.assertTrue(people[0]['velocity_xy_valid'])

    def test_wrong_epoch_invalid_vertical_and_negative_evidence(self):
        self.add('go2',[observation()])
        wrong=observation('uav');wrong['scene_epoch']='wrong'
        with self.assertRaisesRegex(ValueError,'identity'):self.add('uav',[wrong])
        wrong=observation('uav');wrong['velocity_z_valid']=True
        with self.assertRaises(ValueError):self.add('uav',[wrong])
        self.add('go2',[],10.2)
        self.assertEqual(self.registry.snapshot(10.2,1000)['people'],[])

    def test_missing_mask_retires_geometry_but_short_native_continuation_keeps_id(self):
        obs=observation()
        gid=self.add('go2',[obs])[obs['source_key']]
        self.add('go2',[],10.2)
        self.assertEqual(self.registry.snapshot(10.2,1000)['people'],[])
        returned=observation(stamp=10.4)
        self.assertEqual(self.add('go2',[returned],10.4)[returned['source_key']],gid)
        self.add('go2',[],10.5)
        late=observation(stamp=12.)
        self.assertNotEqual(self.add('go2',[late],12.)[late['source_key']],gid)

    def test_drop_one_camera_does_not_drop_other_camera(self):
        self.add('go2',[observation()]);self.add('uav',[observation('uav')])
        self.registry.drop_source(('epoch','go2','front_rgbd'))
        people=self.registry.snapshot(10,1000)['people']
        self.assertEqual(len(people),1);self.assertEqual(len(people[0]['source_keys']),1)
        self.assertIn('uav',people[0]['source_keys'][0])


class SequenceAndSchedulingTests(unittest.TestCase):
    def test_latest_only_round_robin_no_starvation(self):
        scheduler=LatestFrameScheduler(['go2','uav'])
        scheduler.offer('go2','1',1);scheduler.offer('go2','2',2);scheduler.offer('uav','1',3)
        self.assertEqual(scheduler.pop(),('go2',2))
        scheduler.offer('go2','3',4)
        self.assertEqual(scheduler.pop(),('uav',3))
        self.assertEqual(scheduler.pop(),('go2',4));self.assertIsNone(scheduler.pop())
        self.assertEqual(scheduler.replaced['go2'],1)

    def test_idle_source_does_not_break_round_robin(self):
        scheduler=LatestFrameScheduler(['go2','uav'])
        scheduler.offer('uav','1',5)
        self.assertEqual(scheduler.pop(),('uav',5))

    def test_source_sequence_resets_only_actual_discontinuity(self):
        clock=[0.];seq=FrameSequence(lambda:clock[0]);other=FrameSequence()
        self.assertEqual(seq.accept(metadata()),(True,None))
        other.accept(metadata('uav'))
        clock[0]=30.
        self.assertEqual(seq.accept(metadata(stamp=11,frame=2)),(True,None))
        self.assertEqual(seq.accept(metadata(stamp=11,frame=2)),(False,None))
        new=metadata(stamp=12,frame=3);new['calibration_id']='new'
        self.assertEqual(seq.accept(new),(True,'camera_calibration_changed'))
        self.assertEqual(other.last['frame_seq'],1)
        self.assertEqual(seq.accept(metadata(stamp=2,frame=0))[1],'camera_calibration_changed')

    def test_duplicate_token_changed_image_rejected(self):
        seq=FrameSequence();meta=metadata();seq.accept(meta)
        meta['rgb_sha256']='b'*64
        with self.assertRaisesRegex(ValueError,'different_image'):seq.accept(meta)

    def test_backend_reset_is_scoped_without_loading_model(self):
        backend=VideoModel.__new__(VideoModel)
        backend.torch=SimpleNamespace(bfloat16='dtype')
        backend.processor=SimpleNamespace(init_video_session=lambda **kwargs:object(),add_text_prompt=lambda **kwargs:None)
        backend.model=object();backend.sessions={}
        with patch('perception.vendor.sam3_video_memory.configure_streaming_session'):
            a=backend.reset(('epoch','go2','front_rgbd'))
            b=backend.reset(('epoch','uav','front_rgbd'))
            backend.reset(('epoch','go2','front_rgbd'),'calibration_changed')
        self.assertIs(backend.sessions[('epoch','uav','front_rgbd')],b)
        self.assertIsNot(backend.sessions[('epoch','go2','front_rgbd')],a)
        self.assertEqual(b.frame_index,0)


class ServiceTests(unittest.TestCase):
    def test_exact_frame_hash_two_sources_and_missing_input(self):
        class Backend:
            def __init__(self):self.calls=[];self.sessions={}
            def infer(self,key,image):self.calls.append(key);return result(key[1])
            def reset(self,key,reason):pass
            def drop(self,key):pass
        with tempfile.TemporaryDirectory() as temporary:
            root=Path(temporary)
            (root/'resolved_profiles.json').write_text(json.dumps(dict(robots=[dict(robot_id='go2'),dict(robot_id='uav')])))
            state=dict(scene_id='scene',epoch='epoch',sim_time=10.,wall_time=1000.,running=True)
            (root/'state.json').write_text(json.dumps(state))
            for robot in ['go2','uav']:
                folder=root/robot/'rgbd';folder.mkdir(parents=True)
                (folder/'latest.json').write_text(json.dumps(metadata(robot)))
            backend=Backend();service=PeopleService(root,backend,wall=lambda:1000.)
            def load(path, **kwargs):return frame(metadata(path.parent.parent.name))
            with patch('perception.registered_rgbd.load_capture',side_effect=load):
                self.assertTrue(service.step());self.assertTrue(service.step())
            self.assertEqual([key[1] for key in backend.calls],['go2','uav'])
            for robot in ['go2','uav']:
                status=json.loads((root/robot/'sam3/latest.json').read_text())
                image=(root/robot/'sam3/latest.jpg').read_bytes()
                self.assertEqual(status['overlay_sha256'],hashlib.sha256(image).hexdigest())
                self.assertEqual(status['frame_token'],'1'+robot)
                self.assertEqual(status['capture_sim_time'],10.)
                self.assertEqual(status['tracks_count'],1)
            shared=json.loads((root/'shared/people.json').read_text())
            self.assertEqual(len(shared['people']),1)
            (root/'go2/rgbd/latest.json').unlink()
            service.step()
            status=json.loads((root/'go2/sam3/latest.json').read_text())
            self.assertEqual(status['state'],'unavailable');self.assertFalse(status['valid'])
            self.assertEqual(len(service.registry.snapshot(10,1000)['people'][0]['source_keys']),1)

    def test_lagging_clock_defers_without_erasing_history_and_fresh_clock_advances(self):
        with tempfile.TemporaryDirectory() as temporary:
            root=Path(temporary)
            (root/'resolved_profiles.json').write_text(json.dumps(dict(robots=[dict(robot_id='go2')])))
            state=dict(scene_id='scene',epoch='epoch',sim_time=9.5,wall_time=1000.,running=True)
            (root/'state.json').write_text(json.dumps(state))
            (root/'go2/rgbd').mkdir(parents=True)
            (root/'go2/rgbd/latest.json').write_text(json.dumps(metadata()))
            backend=SimpleNamespace(infer=lambda key,image:result(),drop=lambda key:None,sessions={})
            service=PeopleService(root,backend,wall=lambda:1000.)
            key=service.sources[0]
            service.projectors[key].history['retained']=[(9.4,np.zeros(2))]
            self.assertFalse(service.step())
            self.assertIn('retained',service.projectors[key].history)
            self.assertEqual(service.status[key]['input_state'],'awaiting_reference_clock')
            (root/'shared').mkdir(exist_ok=True)
            clock=dict(scene_id='scene',scene_epoch='wrong',sim_time=10.,wall_time=1000.)
            (root/'shared/sensor_clock.json').write_text(json.dumps(clock))
            self.assertEqual(service.read_state()['sim_time'],9.5)
            clock['scene_epoch']='epoch'
            (root/'shared/sensor_clock.json').write_text(json.dumps(clock))
            self.assertEqual(service.read_state()['sim_time'],10.)
            with patch('perception.registered_rgbd.load_capture',return_value=frame()):
                self.assertTrue(service.step())
            self.assertEqual(service.status[key]['state'],'ready')
            # A missed/aged capture is not itself a tracking-session reset.
            service.projectors[key].history['retained']=[(10.,np.zeros(2))]
            clock['sim_time']=11.6
            (root/'shared/sensor_clock.json').write_text(json.dumps(clock))
            service.scan(service.read_state())
            self.assertIn('retained',service.projectors[key].history)
            self.assertEqual(service.registry.snapshot(11.6,1000)['people'],[])

    def test_real_registered_bundle_to_overlay_and_shared_output(self):
        from perception.registered_rgbd import publish_capture
        with tempfile.TemporaryDirectory() as temporary:
            root=Path(temporary)
            (root/'resolved_profiles.json').write_text(json.dumps(dict(robots=[dict(robot_id='go2')])))
            (root/'state.json').write_text(json.dumps(dict(scene_id='scene',epoch='epoch',sim_time=10.,wall_time=1000.,running=True)))
            meta=metadata();meta.update(width=640,height=360,K=[[320.,0,320.],[0,320.,180.],[0,0,1]],
                rgb_frame_seq=1,depth_frame_seq=1,rgb_capture_sim_time=10.,depth_capture_sim_time=10.)
            yy,xx=np.mgrid[0:360,0:640]
            xyz=np.stack(((xx+.5-320.)*3/320,(yy+.5-180.)*3/320,np.full_like(xx,3.)),axis=-1).astype(np.float32)
            rgb=np.full((360,640,3),100,dtype=np.uint8)
            manifest=publish_capture(root/'go2/rgbd',rgb,xyz,meta)
            answer=result();answer['tracks'][0]['box']=[290,100,350,250]
            answer['masks']=np.zeros((1,360,640),dtype=bool);answer['masks'][:,100:250,290:350]=True
            backend=SimpleNamespace(infer=lambda key,image:answer,drop=lambda key:None,sessions={})
            service=PeopleService(root,backend,wall=lambda:1000.)
            self.assertTrue(service.step())
            status=json.loads((root/'go2/sam3/latest.json').read_text())
            self.assertEqual(status['frame_token'],manifest['frame_token'])
            self.assertEqual(status['source_payload_sha256'],manifest['payload_sha256'])
            self.assertEqual(Path(status['source_bundle_path']).name,manifest['file'])
            people=json.loads((root/'shared/people.json').read_text())['people']
            self.assertEqual(len(people),1)
            self.assertEqual(people[0]['capture_sim_time'],10.)
            self.assertFalse(people[0]['velocity_xy_valid'])

    def test_slow_result_is_expired_and_never_enters_shared_people(self):
        with tempfile.TemporaryDirectory() as temporary:
            root=Path(temporary)
            (root/'resolved_profiles.json').write_text(json.dumps(dict(robots=[dict(robot_id='go2')])))
            state=dict(scene_id='scene',epoch='epoch',sim_time=10.,wall_time=1000.,running=True)
            (root/'state.json').write_text(json.dumps(state))
            (root/'go2/rgbd').mkdir(parents=True)
            (root/'go2/rgbd/latest.json').write_text(json.dumps(metadata()))
            def infer(key,image):
                state['sim_time']=12.
                (root/'state.json').write_text(json.dumps(state))
                return result()
            backend=SimpleNamespace(infer=infer,drop=lambda key:None,sessions={})
            service=PeopleService(root,backend,wall=lambda:1000.)
            with patch('perception.registered_rgbd.load_capture',return_value=frame()):service.step()
            output=json.loads((root/'shared/people.json').read_text())
            self.assertEqual(output['people'],[])
            self.assertEqual(output['sources'][0]['state'],'expired')


if __name__=='__main__':unittest.main()
