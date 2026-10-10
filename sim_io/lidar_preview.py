"""Read-only subscription to the two existing LiDAR streams for RViz.

This process never spawns/moves actors or changes navigation. It subscribes using
its own CARLA client; the mapper's independent subscriptions remain running.
"""
import json
import os
from pathlib import Path
import signal
import sys
import threading
import time
import numpy as np
import carla
ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT))
from sim_io.config import carla_port, runtime_home
from sim_io.geometry import transform_points,to_map_xyz

stop=threading.Event()
signal.signal(signal.SIGTERM,lambda *_:stop.set());signal.signal(signal.SIGINT,lambda *_:stop.set())

def main():
    client=carla.Client('127.0.0.1',carla_port());client.set_timeout(3);subscribed={};latest={};lock=threading.Lock();written={};identity=None
    def receive(rid,measurement):
        with lock:latest[rid]=measurement
    try:
        while not stop.is_set():
            try:
                pointer=json.loads((runtime_home()/'latest_run.json').read_text());run=Path(pointer['run_root'])
                profile=json.loads((run/'resolved_profiles.json').read_text());scene=profile['scene_config']
                state=json.loads((run/'state.json').read_text())
                if not state.get('running') or time.time()-state['wall_time']>4:stop.wait(.2);continue
                if identity!=pointer['epoch']:
                    for sensor in subscribed.values():
                        try:sensor.stop()
                        except RuntimeError:pass
                    subscribed={};latest={};written={};identity=pointer['epoch']
                world=client.get_world()
                if world.id!=state['world_id']:stop.wait(.2);continue
                for sensor in world.get_actors().filter('sensor.lidar.ray_cast'):
                    parent=sensor.parent
                    if parent and parent.attributes.get('role_name','').startswith('ag_'):
                        rid=parent.attributes['role_name'][3:]
                        if rid not in {r['robot_id'] for r in profile['robots']} or sensor.id in subscribed:continue
                        sensor.listen(lambda m,rid=rid:receive(rid,m));subscribed[sensor.id]=sensor
                with lock:frames=dict(latest)
                for rid,m in frames.items():
                    if written.get(rid)==m.frame:continue
                    points=np.frombuffer(m.raw_data,np.float32).reshape(-1,4)[:,:3]
                    if len(points)>5000:points=points[np.linspace(0,len(points)-1,5000,dtype=int)]
                    capture_origin=to_map_xyz([m.transform.location.x,m.transform.location.y,m.transform.location.z],scene['carla_world_origin_m'])
                    points=transform_points(points,m.transform.get_matrix(),scene['carla_world_origin_m']).astype(np.float32)
                    folder=run/rid;folder.mkdir(exist_ok=True);target=folder/'latest_lidar.npz';tmp=folder/'latest_lidar.tmp'
                    with tmp.open('wb') as output:np.savez(output,points=points,frame=np.int64(m.frame),capture_sim_time=np.float64(m.timestamp),capture_origin_xyz=capture_origin)
                    os.replace(tmp,target);written[rid]=m.frame
            except (RuntimeError,OSError,KeyError,ValueError) as exc:
                print('Waiting for owned LiDAR:',exc,flush=True);stop.wait(.5)
            stop.wait(.1)
    finally:
        for sensor in subscribed.values():
            try:sensor.stop()
            except RuntimeError:pass

if __name__=='__main__':main()
