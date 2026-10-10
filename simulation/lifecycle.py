"""Explicit lifecycle for this independent experiment; no broad process matching."""
import argparse
import json
from pathlib import Path
import subprocess
import sys
import time

ROOT=Path(__file__).resolve().parents[1]
PYTHON='/data/robotics/ue58_ohz_test_go2/.venv/bin/python'
UNITS=['air-ground-lab-sam3','air-ground-lab-yolo26','air-ground-lab-rgbd-capture','air-ground-lab-lidar-view','air-ground-lab-runtime','air-ground-lab-view','air-ground-lab-rviz','air-ground-lab-simulator']
PEOPLE_UNITS={'sam3':'air-ground-lab-sam3','yolo26':'air-ground-lab-yolo26'}
PEOPLE_CHOICE=ROOT/'runtime/people_backend.json'

def active(name):
    return subprocess.run(['systemctl','--user','is-active','--quiet',name+'.service']).returncode==0


def start_unit(name,command,memory='8G',cwd=None,env=None,stop_timeout='65s'):
    if active(name):return
    commandline=['systemd-run','--user','--unit='+name,'--collect','--property=MemoryMax='+memory,'--property=TimeoutStopSec='+stop_timeout,'--property=KillMode=mixed']
    if cwd:commandline.append('--working-directory='+str(cwd))
    # Tiny 3-column geometry transforms should not create a 64-thread BLAS
    # pool in each robotics sidecar. Keep this setting local to owned services.
    for k,v in ({'OPENBLAS_NUM_THREADS':'1','OMP_NUM_THREADS':'1','MKL_NUM_THREADS':'1',**(env or {})}).items():commandline.append('--setenv='+k+'='+v)
    subprocess.run(commandline+command,check=True)


def people_backend():
    try:name=json.loads(PEOPLE_CHOICE.read_text()).get('backend')
    except (OSError,ValueError,AttributeError):name=None
    return name if name in PEOPLE_UNITS else 'sam3'


def start_people(backend):
    """Exactly one people back end: stop the other one before starting this one."""
    for name,unit in PEOPLE_UNITS.items():
        if name!=backend and active(unit):subprocess.run(['systemctl','--user','stop',unit+'.service'],check=True)
    if backend=='yolo26':
        start_unit(PEOPLE_UNITS['yolo26'],[PYTHON,'-B','-m','perception.service','--backend','yolo26','--max-hz','6','--ttl-seconds','2.0'],
                   memory='6G',cwd=ROOT,stop_timeout='15s',env={'PYTHONDONTWRITEBYTECODE':'1','YOLO_OFFLINE':'1','YOLO_AUTOINSTALL':'False'})
    else:
        start_unit(PEOPLE_UNITS['sam3'],['/data/robotics/sam3_video_people/.venv/bin/python','-B','-m','perception.service','--backend','sam3','--max-hz','2','--ttl-seconds','2.0'],
                   memory='12G',cwd=ROOT,stop_timeout='15s',env={'PYTHONDONTWRITEBYTECODE':'1','HF_HUB_OFFLINE':'1','TRANSFORMERS_OFFLINE':'1'})


def choose_people(backend):
    PEOPLE_CHOICE.parent.mkdir(parents=True,exist_ok=True)
    PEOPLE_CHOICE.write_text(json.dumps({'backend':backend})+'\n')
    # Switch live only while this experiment runs with perception; otherwise the next start uses it.
    if active('air-ground-lab-runtime') and active('air-ground-lab-rgbd-capture'):start_people(backend)


def start(args):
    if args.resume_state and not args.resume_map:raise SystemExit('--resume-state requires --resume-map')
    if args.people:
        PEOPLE_CHOICE.parent.mkdir(parents=True,exist_ok=True)
        PEOPLE_CHOICE.write_text(json.dumps({'backend':args.people})+'\n')
    launch=[sys.executable,str(ROOT/'simulation/launch_sim.py')]
    if args.visible:launch.append('--visible')
    start_unit('air-ground-lab-simulator',launch,memory='30G')
    probe="import carla;c=carla.Client('127.0.0.1',2400);c.set_timeout(2);w=c.get_world();assert w.get_map().name.endswith('/AirGroundLab_Main');print(w.id)"
    for _ in range(45):
        if subprocess.run([PYTHON,'-B','-c',probe],stdout=subprocess.DEVNULL,stderr=subprocess.DEVNULL).returncode==0:break
        time.sleep(2)
    else:raise SystemExit('New simulator is not ready; inspect runtime/simulator.log')
    runtime_command=[PYTHON,'-B','-m','sim_io.runtime','--team',args.team]
    if args.resume_map:runtime_command+=['--resume-map',str(Path(args.resume_map).resolve())]
    if args.resume_state:runtime_command+=['--resume-state',str(Path(args.resume_state).resolve())]
    start_unit('air-ground-lab-runtime',runtime_command,cwd=ROOT,
               env={'PYTHONPATH':str(ROOT),'PYTHONDONTWRITEBYTECODE':'1'})
    if args.rviz:
        start_unit('air-ground-lab-lidar-view',[PYTHON,'-B','-m','sim_io.lidar_preview'],memory='1G',cwd=ROOT,stop_timeout='8s',
                   env={'PYTHONDONTWRITEBYTECODE':'1'})
        start_unit('air-ground-lab-view',['/bin/bash','-c','source /opt/ros/humble/setup.bash && exec /usr/bin/python3 -B -m observability.ros_view'],cwd=ROOT,
                   env={'ROS_DOMAIN_ID':'45','PYTHONDONTWRITEBYTECODE':'1'})
        start_unit('air-ground-lab-rviz',['/bin/bash','-c','source /opt/ros/humble/setup.bash && exec rviz2 -d '+str(ROOT/'observability/air_ground.rviz')+' --ros-args -p use_sim_time:=false'],memory='3G',
                   env={'ROS_DOMAIN_ID':'45','DISPLAY':':1','XAUTHORITY':'/run/user/1000/gdm/Xauthority'})
    # Do not report the previous run's stopped state as the new run's status.
    for _ in range(30):
        try:
            p=Path(json.loads((ROOT/'runtime/latest_run.json').read_text())['run_root'])
            state=json.loads((p/'state.json').read_text())
            if state.get('running') and time.time()-state['wall_time']<4:break
        except (FileNotFoundError,KeyError,ValueError):pass
        time.sleep(1)
    if not args.no_perception:
        start_unit('air-ground-lab-rgbd-capture',[PYTHON,'-B','-m','perception.capture'],memory='2G',cwd=ROOT,stop_timeout='8s',
                   env={'PYTHONDONTWRITEBYTECODE':'1'})
        start_people(people_backend())
    status()


def status():
    report={'units':{name:active(name) for name in UNITS},'people_backend':people_backend()}
    pointer=ROOT/'runtime/latest_run.json'
    if pointer.exists():
        run=Path(json.loads(pointer.read_text())['run_root']);path=run/'state.json'
        if path.exists():
            s=json.loads(path.read_text());report.update(run_root=str(run),age_s=round(time.time()-s['wall_time'],2),map=s['map_summary'],humans=s['human_count'])
            report['robots']=[{k:r.get(k) for k in ('robot_id','dimension','position_xyz','status','completed_goals','distance_m','vertical_distance_m','last_control_error','planner_error')} for r in s['robots']]
            for a,r in zip(report['robots'],s['robots']):a.update(tree_nodes=r.get('planner',{}).get('tree_nodes_count',len(r.get('planner',{}).get('tree_nodes',[]))),tree_id=r.get('planner',{}).get('tree_instance_id'),path_points=len(r.get('planner',{}).get('path_xyz') or []))
    print(json.dumps(report,indent=2))


def main():
    parser=argparse.ArgumentParser();parser.add_argument('action',choices=['start','stop','status','people']);parser.add_argument('backend',nargs='?',choices=sorted(PEOPLE_UNITS),help='with "people": the one people back end to run');parser.add_argument('--team',default='go2_fighter');parser.add_argument('--rviz',action='store_true');parser.add_argument('--visible',action='store_true');parser.add_argument('--no-perception',action='store_true',help='Start geometry baseline without the RGB-D/people services');parser.add_argument('--people',choices=sorted(PEOPLE_UNITS),help='people back end for this and later starts');parser.add_argument('--resume-map',help='Explicit compatible .ot snapshot to load into a new epoch');parser.add_argument('--resume-state',help='Compatible stopped state.json to restore robot poses and patrol progress');args=parser.parse_args()
    if args.action=='start':start(args)
    elif args.action=='people':
        if args.backend:choose_people(args.backend)
        status()
    elif args.action=='stop':
        for name in UNITS:
            if active(name):subprocess.run(['systemctl','--user','stop',name+'.service'],check=True)
        status()
    else:status()

if __name__=='__main__':main()
