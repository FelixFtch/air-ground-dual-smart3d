import copy
import sys
import time
import unittest
from types import SimpleNamespace as NS
from unittest.mock import patch
from sim_io.motion_rpc import SCHEMA, MotionClient, MotionServer

class Location:
    def __init__(self,x=0,y=0,z=0):self.x,self.y,self.z=x,y,z
class Rotation:
    def __init__(self,pitch=0,yaw=0,roll=0):self.pitch,self.yaw,self.roll=pitch,yaw,roll
class Transform:
    def __init__(self,location,rotation):self.location,self.rotation=location,rotation
class ApplyTransform:
    def __init__(self,actor_id,transform):self.actor_id,self.transform=actor_id,transform

class FakeClient:
    def __init__(self):self.world_id=42;self.calls=[];self.get_count=0;self.error_id=None;self.world_hook=None
    def set_timeout(self,value):self.timeout=value
    def get_world(self):
        self.get_count+=1
        if self.world_hook:self.world_hook()
        return NS(id=self.world_id)
    def apply_batch_sync(self,commands,do_tick):
        self.calls.append((commands,do_tick))
        return [NS(actor_id=0,error='UE sweep collision') if c.actor_id==self.error_id else NS(actor_id=c.actor_id,error='') for c in commands]

class MotionTests(unittest.TestCase):
    def setUp(self):
        self.client=FakeClient();self.now=100.
        self.carla=NS(Client=lambda *_:self.client,Location=Location,Rotation=Rotation,Transform=Transform,command=NS(ApplyTransform=ApplyTransform))
        self.server=MotionServer(self.carla,clock=lambda:self.now)
        self.init=dict(schema=SCHEMA,op='init',host='127.0.0.1',port=2400,world_id=42,epoch='ep',allowed_actor_ids=[11,22],rpc_timeout_s=2.)
        self.server.dispatch(self.init)
    def request(self):
        return dict(schema=SCHEMA,op='apply',epoch='ep',world_id=42,sequence=1,deadline_monotonic=101.,transforms=[dict(actor_id=11,xyz_pitch_yaw_roll=[1,2,3,4,5,6]),dict(actor_id=22,xyz_pitch_yaw_roll=[2,3,4,5,6,7])])
    def test_ordered_exact_transforms_and_ue_errors(self):
        self.client.error_id=22;r=self.server.dispatch(self.request())
        self.assertTrue(r['ok']);self.assertEqual([x['command_actor_id'] for x in r['receipts']],[11,22]);self.assertEqual(r['receipts'][1],dict(command_actor_id=22,actor_id=0,error='UE sweep collision'))
        commands,tick=self.client.calls[0];self.assertFalse(tick)
        self.assertEqual(commands[0].transform.rotation.__dict__,dict(pitch=4,yaw=5,roll=6));self.assertEqual(commands[0].transform.location.__dict__,dict(x=1,y=2,z=3))
    def test_wrong_identity_or_sequence_or_expiry_never_dispatches(self):
        for fields in [dict(epoch='wrong'),dict(world_id=43),dict(sequence=0),dict(sequence=2),dict(deadline_monotonic=99.),dict(op='spawn')]:
            with self.subTest(fields=fields):
                req=self.request();req.update(fields)
                with self.assertRaises(ValueError):self.server.dispatch(req)
        self.assertEqual(self.client.calls,[])
    def test_complete_batch_validated_before_dispatch(self):
        for second in [dict(actor_id=33,xyz_pitch_yaw_roll=[0]*6),dict(actor_id=11,xyz_pitch_yaw_roll=[0]*6),dict(actor_id=22,xyz_pitch_yaw_roll=[0]*5),dict(actor_id=22,xyz_pitch_yaw_roll=[0,0,0,0,0,float('nan')])]:
            req=self.request();req['transforms'][1]=second
            with self.assertRaises(ValueError):self.server.dispatch(req)
        self.assertEqual(self.client.calls,[])
    def test_world_change_never_dispatches(self):
        self.client.world_id=43
        with self.assertRaisesRegex(ValueError,'world_mismatch'):self.server.dispatch(self.request())
        self.assertEqual(self.client.calls,[])
    def test_deadline_rechecked_after_world_rpc(self):
        self.client.world_hook=lambda:setattr(self,'now',102.)
        with self.assertRaisesRegex(ValueError,'dispatch_expired'):self.server.dispatch(self.request())
        self.assertEqual(self.client.calls,[])
    def test_replayed_sequence_cannot_repeat_motion(self):
        self.server.dispatch(self.request())
        with self.assertRaisesRegex(ValueError,'sequence_mismatch'):self.server.dispatch(self.request())
        self.assertEqual(len(self.client.calls),1)
    def test_missing_or_wrong_receipts_raise(self):
        for response in [[],[NS(actor_id=99,error=''),NS(actor_id=22,error='')]]:
            self.server.last_sequence=0
            self.client.apply_batch_sync=lambda *_:response
            with self.assertRaisesRegex(ValueError,'receipt'):self.server.dispatch(self.request())
    def test_client_adapter_preserves_receipts_and_rejects_other_commands(self):
        server=MotionServer(self.carla)
        class Pipe:
            closed=False
            def __init__(self,*args,**kwargs):pass
            def call(self,req):return server.dispatch(req)
            def close(self):self.closed=True
        with patch('sim_io.motion_rpc.JsonProcess',Pipe),patch.dict(sys.modules,{'carla':self.carla}):
            wrapper=MotionClient('127.0.0.1',2400,42,'ep',[11,22],'/tmp/fake-log')
            self.client.error_id=22
            commands=[ApplyTransform(i,Transform(Location(1,2,3),Rotation(4,5,6))) for i in [11,22]]
            receipts=wrapper.apply_batch_sync(commands,False)
            self.assertEqual([(r.actor_id,r.error) for r in receipts],[(11,''),(0,'UE sweep collision')])
            with self.assertRaisesRegex(ValueError,'only_apply'):wrapper.apply_batch_sync([NS(actor_id=11,transform=commands[0].transform)])
            with self.assertRaisesRegex(ValueError,'tick_not_allowed'):wrapper.apply_batch_sync(commands,True)
            wrapper._pending.acquire()
            try:
                with self.assertRaisesRegex(RuntimeError,'already_pending'):wrapper.apply_batch_sync(commands)
            finally:wrapper._pending.release()
            pipe=wrapper.worker;wrapper.close();wrapper.close();self.assertTrue(pipe.closed)
