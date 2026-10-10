import json,time,threading,collections,sys
from pathlib import Path
import numpy as np
import carla
sys.path.insert(0,'/data/robotics/air_ground_smart3d')
from sim_io.geometry import transform_points
root=Path('/data/robotics/air_ground_smart3d');run=Path(json.loads((root/'runtime/latest_run.json').read_text())['run_root']);scene=json.loads((run/'resolved_profiles.json').read_text())['scene_config'];origin=scene['carla_world_origin_m'];resolution=scene['resolution']
c=carla.Client('127.0.0.1',2400);c.set_timeout(3);w=c.get_world();walkers=list(w.get_actors().filter('walker.pedestrian.*'));sensors=[s for s in w.get_actors().filter('sensor.lidar.ray_cast') if s.parent and s.parent.attributes.get('role_name','').startswith('ag_')];hist=collections.OrderedDict();lock=threading.Lock();clouds=collections.deque(maxlen=100);results=[]
def tick(snap):
 boxes=[]
 for a in walkers:
  v=snap.find(a.id)
  if v:
   tr=v.get_transform();b=a.bounding_box;corners=np.array([[p.x,p.y,p.z] for p in b.get_world_vertices(tr)]);corners[:,1]*=-1;corners[:,2]-=origin[2];boxes.append((a.id,corners.min(0),corners.max(0)))
 with lock:
  hist[snap.frame]=boxes
  while len(hist)>180:hist.popitem(last=False)
def receive(sid,m):
 with lock:clouds.append((sid,m))
cb=w.on_tick(tick)
for s in sensors:s.listen(lambda m,sid=s.id:receive(sid,m))
start=time.monotonic()
try:
 while time.monotonic()-start<32:
  with lock:
   pending=list(clouds);clouds.clear();history=dict(hist)
  d=json.loads((run/'shared/display.json').read_text());occupied=set(map(tuple,d['occupied']))
  for sid,m in pending:
   boxes=history.get(m.frame)
   if boxes is None:continue
   raw=np.frombuffer(m.raw_data,dtype=np.float32).reshape(-1,4)[:,:3];idx=np.linspace(0,len(raw)-1,min(len(raw),1200),dtype=int);sm=raw[idx];rawmap=transform_points(raw,m.transform.get_matrix(),origin);smmap=rawmap[idx];valid=np.linalg.norm(sm,axis=1)<=12
   for aid,lo,hi in boxes:
    n=int(np.all((rawmap>=lo-.05)&(rawmap<=hi+.05),axis=1).sum());sel=smmap[valid & np.all((smmap>=lo-.05)&(smmap<=hi+.05),axis=1)]
    if not n:continue
    keys=set(map(tuple,np.floor(sel/resolution).astype(int)));ob=sum(k in occupied for k in keys)
    results.append({'sensor':sid,'person':aid,'frame':m.frame,'capture':m.timestamp,'display_time':d['sim_time'],'raw_hits':n,'mapping_sample_hits':len(sel),'sample_hit_voxels':len(keys),'already_occupied_voxels':ob})
  time.sleep(.1)
finally:
 for s in sensors:s.stop()
 w.remove_on_tick(cb)
report={'eval_only':True,'person_truth_not_used_for_mapping':True,'run':str(run),'samples':len(results),'raw_person_hits':sum(x['raw_hits'] for x in results),'mapper_sample_hits':sum(x['mapping_sample_hits'] for x in results),'rows_with_mapping_hits':sum(x['mapping_sample_hits']>0 for x in results),'rows_with_visible_occupied':sum(x['already_occupied_voxels']>0 for x in results),'display_time_lag_mean':float(np.mean([x['capture']-x['display_time'] for x in results])) if results else None,'records':results}
Path('/tmp/air_ground_human_map_probe.json').write_text(json.dumps(report));print(json.dumps({k:v for k,v in report.items() if k!='records'},indent=2))
