"""Launch only this experiment's editor or offscreen CARLA world."""
import argparse
import os
from pathlib import Path
import subprocess

ROOT=Path('/data/robotics/air_ground_smart3d')
PROJECT=Path('/data/sim/AirGroundLab_UE58/CarlaUnreal')
ENGINE=Path('/data/sim/CARLA_UE58_Research/UnrealEngine5_carla58')


def command(prepare=False, visible=False, port=2400, project=None, runtime=None):
    project = Path(project) if project else PROJECT
    runtime = Path(runtime) if runtime else ROOT / 'runtime'
    cmd=[str(ENGINE/'Engine/Binaries/Linux/UnrealEditor'), str(project/'CarlaUnreal.uproject'),
         '/Game/OHZTest/Maps/OHZTest_Main' if prepare else '/Game/AirGroundLab/Maps/AirGroundLab_Main',
         '-carla-rpc-port='+str(port),'-nosplash','-nop4','-NoSound','-unattended',
         '-windowed','-ResX=1280','-ResY=800','-asyncassetcompilationmaxconcurrency=2',
         '-DDC=(Local)','-DDC-NoDefaultGraph','-LocalDataCachePath='+str(runtime/'ddc'),
         '-ini:Engine:[ConsoleVariables]:AssetRegistry.DisableDirectoryWatcher=1',
         '-ini:Engine:[Zen]:AutoLaunch=False','-ini:Engine:[Zen]:SystemService=False',
         '-ini:Engine:[DevOptions.Shaders]:NumUnusedShaderCompilingThreads=26',
         '-ini:EditorPerProjectUserSettings:[/Script/UnrealEd.EditorLoadingSavingSettings]:bMonitorContentDirectories=False',
         '-ExecCmds=carla.Go2.RequireSupport 0,t.MaxFPS 30,r.Streaming.PoolSize 1800,r.Nanite.Streaming.StreamingPoolSize 512',
         '-abslog='+str(runtime/'simulator.log')]
    if port != 2400:
        cmd += ['-WinX=40', '-WinY=40']
    if prepare:
        cmd += ['-ModelContextProtocolStartServer','-ModelContextProtocolPort=18183',
                '-ExecutePythonScript='+str(ROOT/'simulation/prepare_scene.py')]
    else:
        cmd += ['-game']
        if not visible:cmd += ['-RenderOffScreen']
    return cmd


if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--prepare',action='store_true');p.add_argument('--visible',action='store_true');p.add_argument('--print',action='store_true');p.add_argument('--port',type=int,default=2400);p.add_argument('--project',default=str(PROJECT));p.add_argument('--runtime',default=str(ROOT/'runtime'));args=p.parse_args()
    cmd=command(args.prepare,args.visible,port=args.port,project=args.project,runtime=args.runtime)
    if args.print:print(cmd);raise SystemExit
    runtime=Path(args.runtime)
    env=os.environ.copy()
    for k in ['PYTHONPATH','PYTHONHOME','LD_LIBRARY_PATH','AMENT_PREFIX_PATH','COLCON_PREFIX_PATH','CMAKE_PREFIX_PATH']:env.pop(k,None)
    env.update(DISPLAY=':1',XAUTHORITY='/run/user/1000/gdm/Xauthority',XDG_RUNTIME_DIR='/run/user/1000',
               DLSS_SDK='/data/sim/CARLA_UE58_Research/SDKs/DLSS',UE_SHARED_DATA_CACHE_PATH='None',
               XDG_CACHE_HOME=str(runtime/'xdg-cache'),XDG_CONFIG_HOME=str(runtime/'xdg-config'))
    for name in ['ddc','xdg-cache','xdg-config']: (runtime/name).mkdir(parents=True,exist_ok=True)
    raise SystemExit(subprocess.call(cmd,cwd=Path(args.project),env=env))
