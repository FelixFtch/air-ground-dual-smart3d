#!/usr/bin/env python3
"""Read-only RViz view of the current air/ground run; never controls actors.

ROS imports are delayed so contract/geometry functions can be tested offline.
Markers and TF use a common publication-time wall stamp. Actual pose/paths always
come from the saved runtime state, with simulation time displayed explicitly.
"""
from __future__ import annotations

import argparse
import hashlib
import json
import math
import os
from pathlib import Path
import re
import time

from observability.tree_snapshots import TreeSnapshotReader, geometry_digest, reference_signature


class AtomicJsonCache:
    """Decode changed atomic snapshots only; a failed read is never cached."""
    def __init__(self):
        self.entries = {}

    def read(self, path):
        path = Path(path)
        with path.open("rb") as stream:
            stat = os.fstat(stream.fileno())
            signature = (stat.st_ino, stat.st_mtime_ns, stat.st_size)
            cached = self.entries.get(path)
            if cached is not None and cached[0] == signature:
                return cached[1]
            self.entries.pop(path, None)
            document = json.load(stream)
        self.entries[path] = (signature, document)
        return document

    def clear(self):
        self.entries.clear()


class MarkerDeltaTracker:
    """Per-topic marker identity; destructive operations are never heartbeats."""
    def __init__(self):
        self.keys = None
        self.epoch = None

    def update(self, keys, epoch):
        current = frozenset(keys)
        first = self.keys is None
        # A new scene must remove the old markers even when it reuses IDs.
        removed = (() if first else tuple(sorted(
            self.keys if epoch != self.epoch else self.keys - current)))
        self.keys = current
        self.epoch = epoch
        return first, removed


PROJECT_ROOT = Path(__file__).resolve().parents[1]
COLORS = ((0.12, .78, 1.), (1., .62, .12), (.58, .9, .32), (.95, .38, .83))


def xyz(value):
    if not isinstance(value, (list, tuple)) or len(value) != 3:
        return None
    try:
        result = tuple(float(v) for v in value)
    except (TypeError, ValueError):
        return None
    return result if all(math.isfinite(v) for v in result) else None


def actual_tree_edges(planner, max_edges=30000):
    """Only exporter-supplied edges whose parent IDs really match are displayed."""
    if planner.get("tree_exported") is False:
        return {"main": [], "disconnected": [], "pruned": []}, 0
    nodes = {}
    duplicates = set()
    for node in planner.get("tree_nodes") or []:
        if not isinstance(node, dict) or not isinstance(node.get("id"), int):
            continue
        position = xyz(node.get("xyz"))
        if position is None:
            continue
        identifier = node["id"]
        if identifier in nodes:
            duplicates.add(identifier)
        nodes[identifier] = (position, node.get("parent"), node.get("tree_id", 0))
    result = {"main": [], "disconnected": [], "pruned": []}
    omitted = 0
    accepted = 0
    seen = set()
    for edge in planner.get("tree_edges") or []:
        if not isinstance(edge, (list, tuple)) or len(edge) != 2:
            omitted += 1
            continue
        child_id, parent_id = edge
        if not isinstance(child_id, int) or not isinstance(parent_id, int):
            omitted += 1
            continue
        if (child_id, parent_id) in seen:
            continue
        seen.add((child_id, parent_id))
        child, parent = nodes.get(child_id), nodes.get(parent_id)
        if (child is None or parent is None or child_id in duplicates or parent_id in duplicates
                or child_id == parent_id or child[1] != parent_id):
            omitted += 1
            continue
        if accepted >= max_edges:
            omitted += 1
            continue
        kind = "pruned" if child[2] == -1 else ("main" if child[2] == 0 else "disconnected")
        result[kind].extend((child[0], parent[0]))
        accepted += 1
    return result, omitted


def occupancy_projection(display, max_cells=2_000_000):
    """Convert root's explicitly supplied XY projection; do not infer free from Z."""
    resolution = float(display.get("resolution", .25))
    if not math.isfinite(resolution) or resolution <= 0:
        raise ValueError("invalid projection resolution")
    free, blocked = set(), set()
    for field, target in (("free_xy", free), ("blocked_xy", blocked)):
        for key in display.get(field, []):
            if (isinstance(key, (list, tuple)) and len(key) == 2
                    and all(isinstance(v, int) and not isinstance(v, bool) for v in key)):
                target.add(tuple(key))
    cells = free | blocked
    if not cells:
        return None
    xmin, xmax = min(k[0] for k in cells), max(k[0] for k in cells)
    ymin, ymax = min(k[1] for k in cells), max(k[1] for k in cells)
    width, height = xmax - xmin + 1, ymax - ymin + 1
    if width * height > max_cells:
        raise ValueError("projection exceeds display cell budget")
    data = [-1] * (width * height)
    for x, y in free:
        data[(y-ymin)*width+x-xmin] = 0
    for x, y in blocked:
        data[(y-ymin)*width+x-xmin] = 100
    return {"resolution": resolution, "width": width, "height": height,
            "origin": [xmin*resolution, ymin*resolution, float(display.get("ground_z", 0))],
            "data": data}


def read_runtime(pointer, stale_after_s=3., *, cache=None, include_display=True):
    """Read bounded atomic snapshots; transient incomplete writes become a status."""
    read = cache.read if cache is not None else lambda path: json.loads(Path(path).read_text())
    latest = read(pointer)
    run_root = Path(latest["run_root"])
    if not run_root.is_absolute():
        raise ValueError("latest_run.run_root must be absolute")
    state_path = run_root / "state.json"
    state = dict(read(state_path))
    state["_view_run_root"] = str(run_root)
    state_age = max(0., time.time()-state_path.stat().st_mtime)
    state["_view_state_age_s"] = state_age
    display_path = run_root / "shared/display.json"
    display = None
    issues = []
    if not include_display:
        if state_age > stale_after_s:
            issues.append(f"STALE state {state_age:.1f}s")
        return state, None, issues
    if display_path.is_file():
        try:
            candidate = read(display_path)
            if candidate.get("scene_epoch", candidate.get("epoch")) != state.get("epoch"):
                issues.append("map epoch mismatch; map hidden")
            elif candidate.get("scene_id", state.get("scene_id")) != state.get("scene_id"):
                issues.append("map scene mismatch; map hidden")
            else:
                display = candidate
        except (OSError, ValueError, TypeError):
            issues.append("map snapshot unavailable")
    else:
        issues.append("waiting for shared/display.json")
    if state_age > stale_after_s:
        issues.append(f"STALE state {state_age:.1f}s")
    return state, display, issues


def robot_image_files(state):
    """Only files from the same resolved run as state, never another episode."""
    if not state.get("_view_run_root"):
        return {}
    root=Path(state["_view_run_root"])
    result={}
    for robot in state.get("robots") or []:
        rid=robot.get("robot_id","")
        if not isinstance(rid,str) or not re.fullmatch(r"[A-Za-z][A-Za-z0-9_]*",rid):
            continue
        candidates=[root/rid/"sensor/latest_rgb.jpg",root/rid/"latest_rgb.jpg"]
        candidates=[path for path in candidates if path.is_file()]
        if candidates:
            result[rid]=max(candidates,key=lambda path:path.stat().st_mtime_ns)
    return result


def read_lidar_frame(path):
    """Validate a single atomically written measured cloud, already in map XYZ."""
    import numpy as np
    with np.load(path,allow_pickle=False) as data:
        points=np.asarray(data["points"])
        frame_value=data["frame"]
        stamp_value=data["capture_sim_time"]
        if frame_value.shape!=() or stamp_value.shape!=():
            raise ValueError("lidar frame and capture_sim_time must be scalars")
        frame=frame_value.item();stamp=float(stamp_value.item())
        if isinstance(frame,bool) or not isinstance(frame,int) or frame<0 or not math.isfinite(stamp) or stamp<0:
            raise ValueError("invalid lidar capture identity")
        if points.ndim!=2 or points.shape[1]!=3 or len(points)>250000:
            raise ValueError("lidar points must be a bounded Nx3 array")
        if not np.issubdtype(points.dtype,np.number) or not np.isfinite(points).all():
            raise ValueError("lidar points must be finite measured XYZ")
        points=np.ascontiguousarray(points,dtype="<f4")
    return frame,stamp,points


def scene_specs(state, display, *, max_edges=30000, max_voxels=100000, issues=(), include_trees=True):
    """ROS-independent representation of truthful marker content."""
    markers = {"robots": [], "paths": [], "trees": [], "map": [], "people": []}
    transforms = []
    robot_ids = set()
    extra = list(issues)
    for index, robot in enumerate(state.get("robots", [])):
        identifier = robot.get("robot_id", "")
        if not isinstance(identifier, str) or not re.fullmatch(r"[A-Za-z][A-Za-z0-9_]*", identifier) or identifier in robot_ids:
            extra.append("invalid or duplicate robot ID omitted")
            continue
        robot_ids.add(identifier)
        position = xyz(robot.get("position_xyz"))
        half = xyz(robot.get("body_half_extents_m"))
        if position is None or half is None or any(v <= 0 for v in half):
            extra.append(f"{identifier}: invalid pose/body omitted")
            continue
        yaw = float(robot.get("yaw", 0.))
        if not math.isfinite(yaw):
            extra.append(f"{identifier}: invalid yaw omitted")
            continue
        offset = xyz(robot.get("body_offset_m", [0, 0, 0])) or (0, 0, 0)
        center = tuple(a+b for a, b in zip(position, offset))
        fixed_colors={"go2_01":COLORS[0],"uav_01":COLORS[1],"go2_02":COLORS[2],"uav_02":COLORS[3]}
        color = fixed_colors.get(identifier, COLORS[index % len(COLORS)])
        transforms.append({"child": f"{identifier}/base_link", "position": position, "yaw": yaw})
        markers["robots"].append(dict(ns=f"{identifier}/body_envelope", kind="cube", position=center,
                                       scale=[v*2 for v in half], color=color, alpha=.55))
        locator_top = (position[0], position[1], position[2]+2.)
        markers["robots"].append(dict(ns=f"{identifier}/position_locator", kind="line_list",
                                       points=[position,locator_top],scale=[.085,0,0],color=color,alpha=1.))
        markers["robots"].append(dict(ns=f"{identifier}/name",kind="text",
                                       position=[position[0],position[1],position[2]+2.45],
                                       scale=[0,0,.52],color=color,alpha=1.,text=identifier.upper()))
        arrow_end = (position[0]+1.35*math.cos(yaw), position[1]+1.35*math.sin(yaw), position[2])
        markers["robots"].append(dict(ns=f"{identifier}/heading", kind="arrow", points=[position, arrow_end],
                                       scale=[.12, .26, .30], color=color, alpha=1.))
        goal = xyz(robot.get("goal_xyz"))
        if goal:
            markers["robots"].append(dict(ns=f"{identifier}/goal", kind="sphere", position=goal,
                                           scale=[.22]*3, color=color, alpha=.8))
        planner = robot.get("planner") or {}
        radius=planner.get('local_reaction_radius_m')
        lrz_center=xyz(planner.get('local_reaction_center_xyz'))
        if isinstance(radius,(int,float)) and 0<radius<100 and lrz_center is not None:
            if robot.get('dimension')==2:
                circle=[(lrz_center[0]+radius*math.cos(i*math.tau/96),lrz_center[1]+radius*math.sin(i*math.tau/96),lrz_center[2]) for i in range(97)]
                markers['people'].append(dict(ns=identifier+'/native_lrz',kind='line_strip',points=circle,scale=[.035,0,0],color=color,alpha=.7))
            else:
                markers['people'].append(dict(ns=identifier+'/native_lrz',kind='sphere',position=lrz_center,scale=[2*radius]*3,color=color,alpha=.035))
        path = [xyz(p) for p in planner.get("path_xyz") or []]
        # Reject a malformed whole path instead of bridging the missing point.
        if path and all(p is not None for p in path):
            markers["paths"].append(dict(ns=f"{identifier}/actual_path", kind="line_strip", points=path,
                                          scale=[.13, 0, 0], color=color, alpha=1.))
        tree, omitted = (actual_tree_edges(planner, max_edges=max_edges) if include_trees
                         else ({"main": [], "disconnected": [], "pruned": []}, 0))
        for kind, points in tree.items():
            if points:
                markers["trees"].append(dict(ns=f"{identifier}/tree/{kind}", kind="line_list", points=points,
                                              scale=[.025 if kind=="main" else .02, 0, 0],
                                              color=color if kind=="main" else (.65,.65,.65),
                                              alpha=.35 if kind=="main" else .20))
        count = sum(len(points)//2 for points in tree.values())
        tree_note = (f"{count} real edges" if include_trees else "native tree") if planner.get("tree_exported", True) else "tree not exported"
        if omitted:
            tree_note += f"; {omitted} omitted"
        labels = f"{robot.get('status', 'unknown')} | body envelope\nSMART {tree_note}"
        markers["robots"].append(dict(ns=f"{identifier}/label", kind="text", position=[position[0],position[1],position[2]+2.05],
                                       scale=[0,0,.20], color=color, alpha=1., text=labels))
    for robot in state.get('robots',[]):
        for hazard in robot.get('people_hazards',[]):
            if hazard.get('valid_until_sim_time',-1)<=state.get('sim_time',0):continue
            try:
                from perception.hazards import validate_hazard
                validate_hazard(hazard)
            except (ValueError,TypeError,KeyError):continue
            rid=robot['robot_id'];label=hazard['id'];center=hazard['center_xyz'];axes=hazard['axes_xyz'];radii=hazard['semi_axes_m']
            yaw=math.atan2(axes[0][1],axes[0][0]);dim=hazard['dimension']
            color,alpha=(.25,.95,.48),.19 if dim==2 else (.85,.42,.08),.45
            kind='cube' if hazard.get('shape')=='box' else 'sphere'
            markers['people'].append(dict(ns=rid+'/ohz/'+label,kind=kind,position=center,
                scale=[2*radii[0],2*radii[1],.08 if dim==2 else 2*radii[2]],yaw=yaw,color=color,alpha=alpha))
    people=state.get('_people_document') or {}
    for person in people.get('people',[]):
        if person.get('valid_until_sim_time',-1)<=state.get('sim_time',0):continue
        center=xyz(person.get('center_xyz'));velocity=xyz(person.get('velocity_xyz'))
        if center is None or velocity is None:continue
        pid=str(person.get('global_id',person.get('id','unknown')))
        measured=xyz(person.get('measured_center_xyz')) or center
        half=xyz(person.get('measured_half_extents_xyz')) or xyz(person.get('half_extents_xyz'))
        if half is not None:
            lo=[measured[i]-half[i] for i in range(3)];hi=[measured[i]+half[i] for i in range(3)]
            corners=[(x,y,z) for x in (lo[0],hi[0]) for y in (lo[1],hi[1]) for z in (lo[2],hi[2])]
            edges=[]
            for i,a in enumerate(corners):
                for j,b in enumerate(corners):
                    if j>i and sum(a[k]!=b[k] for k in range(3))==1:edges.extend([a,b])
            markers['people'].append(dict(ns='people/'+pid+'/measured_surface_box',kind='line_list',points=edges,scale=[.045,0,0],color=(.2,1.,.5),alpha=.95))
        markers['people'].append(dict(ns='people/'+pid+'/position',kind='sphere',position=center,
            scale=[.18]*3,color=(.25,.95,.48),alpha=1.))
        if person.get('velocity_xy_valid'):
            end=[center[i]+velocity[i] for i in range(3)]
            markers['people'].append(dict(ns='people/'+pid+'/velocity',kind='arrow',points=[center,end],scale=[.08,.16,.20],color=(.3,1.,.4),alpha=1.))
        speed=math.hypot(*velocity[:2]);note=(f'{speed:.2f} m/s' if person.get('velocity_xy_valid') else 'speed collecting')
        markers['people'].append(dict(ns='people/'+pid+'/label',kind='text',position=[center[0],center[1],center[2]+1.1],
            scale=[0,0,.23],color=(.45,1.,.65),alpha=1.,text=pid+' | '+note+' | '+str(len(person.get('source_keys',[])))+' views'))
    if display is not None:
        resolution = float(display.get("resolution", .25))
        if not math.isfinite(resolution) or resolution <= 0:
            raise ValueError("invalid voxel resolution")
        occupied = display.get("occupied", [])
        stride = max(1, math.ceil(len(occupied)/max_voxels))
        ground_z = float(display.get("ground_z",0.))
        if not math.isfinite(ground_z):
            raise ValueError("invalid ground height")
        ground_key = math.floor(ground_z/resolution)
        floor_points, obstacle_points = [], []
        for key in occupied[::stride]:
            if isinstance(key, (list,tuple)) and len(key)==3 and all(isinstance(v,int) for v in key):
                points = floor_points if key[2]<=ground_key else obstacle_points
                points.append([(v+.5)*resolution for v in key])
        if floor_points:
            markers["map"].append(dict(ns="shared/occupied_floor",kind="cube_list",points=floor_points,
                                       scale=[resolution]*3,color=(.48,.51,.55),alpha=.12))
        if obstacle_points:
            markers["map"].append(dict(ns="shared/occupied_obstacles",kind="cube_list",points=obstacle_points,
                                       scale=[resolution]*3,color=(.92,.23,.18),alpha=.55))
        if stride>1:
            extra.append(f"map display sampled 1/{stride}; authority unchanged")
    revision = display.get("revision", "?") if display else (state.get("map_summary") or {}).get("revision", "?")
    summary=f"{state.get('scene_id','?')} / {state.get('epoch','?')}\nsim {state.get('sim_time','?')} | shared map r{revision}"
    if extra:
        summary += "\n" + " | ".join(extra)
    markers["robots"].append(dict(ns="shared/status",kind="text",position=[0,0,7],scale=[0,0,.27],
                                   color=(1.,.9,.55),alpha=1.,text=summary))
    return markers, transforms



def tree_snapshot_signature(state):
    """Geometry identity only; paths and response timestamps cannot rebuild it."""
    if state is None:
        return None
    signatures=[]
    for robot in state.get("robots", []):
        planner=robot.get("planner") or {}
        if "_tree_display_identity" in planner:
            geometry=planner["_tree_display_identity"]
        elif "tree_ref" in planner:
            geometry=reference_signature(planner.get("tree_ref"))
        else:
            # Compatibility during rolling deployment of the old publisher.
            # This function is called exclusively by the 2 Hz tree timer.
            try:
                geometry=geometry_digest(planner)
            except (ValueError,TypeError):
                geometry=("invalid_inline_tree",)
        signatures.append((robot.get("robot_id"), planner.get("session_id"),
                           planner.get("tree_instance_id"), planner.get("tree_exported"), geometry))
    return state.get("epoch"), tuple(signatures)

def main(argv=None):
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--latest-run",type=Path,default=PROJECT_ROOT/"runtime/latest_run.json")
    parser.add_argument("--domain-id",type=int,default=45)
    parser.add_argument("--rate-hz",type=float,default=10.,help="Robot, path and person polling Hz (cached snapshots)")
    parser.add_argument("--map-rate-hz",type=float,default=5.)
    parser.add_argument("--tree-rate-hz",type=float,default=2.)
    parser.add_argument("--max-tree-edges",type=int,default=30000)
    parser.add_argument("--max-display-voxels",type=int,default=100000)
    args=parser.parse_args(argv)
    if not 0 <= args.domain_id <= 101:
        parser.error("ROS domain id must be 0..101")
    if (args.rate_hz<=0 or args.map_rate_hz<=0 or args.tree_rate_hz<=0
            or args.max_tree_edges<=0 or args.max_display_voxels<=0):
        parser.error("Rates and display limits must be positive")
    os.environ["ROS_DOMAIN_ID"]=str(args.domain_id)
    import rclpy
    from rclpy.node import Node
    from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
    from rclpy.executors import MultiThreadedExecutor
    from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
    from builtin_interfaces.msg import Duration
    from geometry_msgs.msg import Point, TransformStamped
    from nav_msgs.msg import OccupancyGrid
    from sensor_msgs.msg import Image, PointCloud2, PointField
    from tf2_ros import TransformBroadcaster
    from visualization_msgs.msg import Marker, MarkerArray

    class View(Node):
        def __init__(self):
            super().__init__("air_ground_readonly_view")
            qos=QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL,reliability=ReliabilityPolicy.RELIABLE)
            topics={"robots":"robot_markers","paths":"path_markers","trees":"tree_markers","map":"shared_map_markers","people":"people_markers"}
            self.publishers_by_kind={key:self.create_publisher(MarkerArray,"/air_ground/"+topic,qos) for key,topic in topics.items()}
            self.projection_pub=self.create_publisher(OccupancyGrid,"/air_ground/ground_projection",qos)
            self.tf=TransformBroadcaster(self)
            self.last_error=None
            self.image_publishers={}
            self.image_signatures={}
            self.image_qos=qos
            self.cloud_publishers={}
            self.cloud_signatures={}
            self.cloud_frames={}
            self.cloud_errors={}
            self.active_run_root=None
            self.active_robot_ids=[]
            self.cloud_qos=QoSProfile(depth=1,durability=DurabilityPolicy.VOLATILE,
                                     reliability=ReliabilityPolicy.BEST_EFFORT)
            self.state_cache=AtomicJsonCache()
            self.map_cache=AtomicJsonCache()
            self.state=None
            self.tree_signature=None
            self.tree_reader=TreeSnapshotReader()
            self.tree_error=None
            self.map_signature=None
            self.cached_arrays={}
            self.marker_deltas={key:MarkerDeltaTracker() for key in topics}
            self.cached_projection=None
            self.last_published={}
            # Large maps / trees must not serialize on the LiDAR timer or pose
            # callback. Separate mutually-exclusive groups own their caches.
            self.groups=[MutuallyExclusiveCallbackGroup() for _ in range(5)]
            self.create_timer(1./args.rate_hz,self.tick,callback_group=self.groups[0])
            self.create_timer(1./args.map_rate_hz,self.maps,callback_group=self.groups[1])
            self.create_timer(1./args.tree_rate_hz,self.trees,callback_group=self.groups[2])
            self.create_timer(.2,self.cameras,callback_group=self.groups[3])
            self.create_timer(.1,self.pointclouds,callback_group=self.groups[4])

        def pointclouds(self):
            # Dedicated 10 Hz display path: never re-read the full planner tree
            # or infer no-return rays, and never pass displayed clouds to control.
            if self.active_run_root is None:
                return
            for rid in self.active_robot_ids:
                path=self.active_run_root/rid/"latest_lidar.npz"
                try:
                    stat=path.stat();signature=(str(path),stat.st_mtime_ns,stat.st_size)
                    if signature==self.cloud_signatures.get(rid):
                        continue
                    frame,capture_time,points=read_lidar_frame(path)
                    identity=(str(path),frame,capture_time)
                    self.cloud_signatures[rid]=signature
                    if identity==self.cloud_frames.get(rid):
                        continue
                    if rid not in self.cloud_publishers:
                        self.cloud_publishers[rid]=self.create_publisher(PointCloud2,f"/air_ground/{rid}/lidar/points",self.cloud_qos)
                    message=PointCloud2();message.header.frame_id="map"
                    message.header.stamp=self.get_clock().now().to_msg()
                    message.height=1;message.width=len(points);message.is_bigendian=False
                    message.fields=[PointField(name=axis,offset=i*4,datatype=PointField.FLOAT32,count=1)
                                    for i,axis in enumerate(("x","y","z"))]
                    message.point_step=12;message.row_step=len(points)*12
                    message.is_dense=True;message.data=points.tobytes()
                    self.cloud_publishers[rid].publish(message)
                    self.cloud_frames[rid]=identity;self.cloud_errors.pop(rid,None)
                except FileNotFoundError:
                    continue
                except (OSError,ValueError,KeyError,TypeError) as error:
                    if self.cloud_errors.get(rid)!=str(error):
                        self.get_logger().warning(f"{rid} measured cloud waiting: {error}")
                        self.cloud_errors[rid]=str(error)

        def images(self,state,issues):
            try:
                import cv2
            except ImportError:
                issues.append("camera display unavailable: cv2 missing")
                return
            files=robot_image_files(state)
            for rid,path in files.items():
                stat=path.stat()
                age=max(0.,time.time()-stat.st_mtime)
                if age>3:
                    issues.append(f"{rid} RGB file STALE {age:.1f}s")
                signature=(str(path),stat.st_mtime_ns,stat.st_size)
                if signature==self.image_signatures.get(rid):
                    continue
                bgr=cv2.imread(str(path),cv2.IMREAD_COLOR)
                if bgr is None:
                    issues.append(f"{rid} RGB write pending")
                    continue
                if rid not in self.image_publishers:
                    self.image_publishers[rid]=self.create_publisher(Image,f"/air_ground/{rid}/rgb/image",self.image_qos)
                message=Image();message.header.frame_id=f"{rid}/rgb_camera"
                # File receipt time only: never pretend an unpaired sidecar is
                # the captured RGB-D timestamp. Geometry never consumes this.
                message.header.stamp.sec=int(stat.st_mtime)
                message.header.stamp.nanosec=int((stat.st_mtime%1)*1e9)
                message.height,message.width=bgr.shape[:2];message.encoding="bgr8"
                message.is_bigendian=0;message.step=message.width*3;message.data=bgr.tobytes()
                self.image_publishers[rid].publish(message)
                self.image_signatures[rid]=signature

        def people_images(self,state,issues):
            import cv2
            import numpy as np
            root=Path(state['_view_run_root'])
            for robot in state.get('robots',[]):
                rid=robot['robot_id']
                # One people back end runs at a time; show whichever wrote this camera's status last.
                latest=None
                for backend in ('sam3','yolo26'):
                    try:meta=json.loads((root/rid/backend/'latest.json').read_text())
                    except (OSError,ValueError):continue
                    if meta.get('scene_epoch')!=state['epoch'] or meta.get('robot_id')!=rid:continue
                    if latest is None or meta.get('generated_wall_time',0)>latest[1].get('generated_wall_time',0):latest=(backend,meta)
                if latest is None:continue
                backend,meta=latest
                try:
                    data=(root/rid/backend/'latest.jpg').read_bytes()
                    if hashlib.sha256(data).hexdigest()!=meta.get('overlay_sha256'):continue
                    key=rid+'/people';signature=(state['epoch'],backend,meta['frame_token'],meta['overlay_sha256'])
                    if signature==self.image_signatures.get(key):continue
                    bgr=cv2.imdecode(np.frombuffer(data,dtype=np.uint8),cv2.IMREAD_COLOR)
                    if bgr is None:continue
                    age=state['sim_time']-meta['capture_sim_time']
                    cv2.putText(bgr,f"{backend.upper()} {rid} age={age:.1f}s {meta.get('state','?')}",(6,22),cv2.FONT_HERSHEY_SIMPLEX,.45,(255,255,255),1)
                    if key not in self.image_publishers:self.image_publishers[key]=self.create_publisher(Image,f'/air_ground/{rid}/people/image',self.image_qos)
                    message=Image();message.header.frame_id=rid+'/rgb_camera';message.header.stamp=self.get_clock().now().to_msg()
                    message.height,message.width=bgr.shape[:2];message.encoding='bgr8';message.step=message.width*3;message.data=bgr.tobytes()
                    self.image_publishers[key].publish(message);self.image_signatures[key]=signature
                except (OSError,KeyError,ValueError,TypeError):continue

        def marker(self,spec,stamp):
            marker=Marker(); marker.header.frame_id="map"; marker.header.stamp=stamp
            marker.ns=spec["ns"]; marker.id=0; marker.action=Marker.ADD
            marker.type={"cube":Marker.CUBE,"sphere":Marker.SPHERE,"arrow":Marker.ARROW,
                         "text":Marker.TEXT_VIEW_FACING,"line_list":Marker.LINE_LIST,
                         "line_strip":Marker.LINE_STRIP,"cube_list":Marker.CUBE_LIST}[spec["kind"]]
            marker.pose.orientation.w=math.cos(spec.get('yaw',0)/2);marker.pose.orientation.z=math.sin(spec.get('yaw',0)/2)
            marker.pose.position.x,marker.pose.position.y,marker.pose.position.z=map(float,spec.get("position",[0,0,0]))
            marker.scale.x,marker.scale.y,marker.scale.z=map(float,spec["scale"])
            marker.color.r,marker.color.g,marker.color.b=spec["color"];marker.color.a=spec["alpha"]
            marker.lifetime=Duration(sec=3)
            marker.text=spec.get("text","")
            for values in spec.get("points",[]):
                p=Point();p.x,p.y,p.z=map(float,values);marker.points.append(p)
            return marker

        def publish_specs(self,kind,specs,stamp,changed=True,epoch=None):
            # Current ADD messages alone are cached. Replaying DELETEALL on
            # each map/tree heartbeat needlessly destroys RViz geometry.
            now=time.monotonic()
            tracker=self.marker_deltas[kind]
            changed=changed or tracker.keys is None or tracker.epoch!=epoch
            if not changed and kind!="trees" and now-self.last_published.get(kind,0.)<1.:
                return
            prefix=[]
            if changed or kind not in self.cached_arrays:
                additions=[self.marker(s,stamp) for s in specs]
                clear_all,removed=tracker.update(((m.ns,m.id) for m in additions),epoch)
                if clear_all:
                    clear=Marker();clear.action=Marker.DELETEALL;clear.header.frame_id="map"
                    prefix.append(clear)
                else:
                    for namespace,identifier in removed:
                        deletion=Marker();deletion.action=Marker.DELETE
                        deletion.header.frame_id="map";deletion.ns=namespace;deletion.id=identifier
                        prefix.append(deletion)
                self.cached_arrays[kind]=MarkerArray(markers=additions)
            message=self.cached_arrays[kind]
            for marker in message.markers:
                marker.header.stamp=stamp
            if prefix:
                for marker in prefix:
                    marker.header.stamp=stamp
                message=MarkerArray(markers=prefix+message.markers)
            self.publishers_by_kind[kind].publish(message)
            self.last_published[kind]=now

        def cameras(self):
            state=self.state
            if state is not None:
                try:
                    self.images(state,[])
                    self.people_images(state,[])
                except (OSError,ValueError,KeyError,TypeError):
                    # The sidecar may rotate files while we read; next tick
                    # samples the newest frame, without stopping geometry.
                    pass

        def trees(self):
            # Large immutable files are opened only here; the fast pose/path
            # timer reads a compact state and never traverses native tree data.
            try:
                state,issues=self.tree_reader.hydrate(self.state)
            except (OSError,ValueError,KeyError,TypeError):
                state,issues=None,["invalid tree state hidden"]
                self.tree_reader.hydrate(None)
            error="; ".join(issues) or None
            if error!=self.tree_error:
                if error:self.get_logger().warning(error)
                self.tree_error=error
            signature=tree_snapshot_signature(state)
            changed=signature!=self.tree_signature or "trees" not in self.cached_arrays
            specs=[]
            if changed and state is not None:
                try:
                    groups,_=scene_specs(state,None,max_edges=args.max_tree_edges)
                    specs=groups["trees"]
                except (ValueError,KeyError,TypeError):
                    specs=[]
            # Healthy identical geometry reuses MarkerArray but still ADDs at
            # 2 Hz. Failure/epoch/session change removes obsolete IDs promptly.
            self.publish_specs("trees",specs,self.get_clock().now().to_msg(),changed,
                               epoch=state.get("epoch") if state is not None else None)
            self.tree_signature=signature

        def maps(self):
            state=self.state
            display=None
            signature=None
            if state is not None:
                path=Path(state["_view_run_root"])/"shared/display.json"
                try:
                    candidate=self.map_cache.read(path)
                    if (candidate.get("scene_epoch",candidate.get("epoch"))==state.get("epoch")
                            and candidate.get("scene_id",state.get("scene_id"))==state.get("scene_id")):
                        display=candidate
                        # File identity also handles a publisher correcting a
                        # display projection without advancing map revision.
                        signature=(state.get("epoch"),self.map_cache.entries[path][0])
                except (OSError,ValueError,KeyError,TypeError):
                    pass
            changed=signature!=self.map_signature or "map" not in self.cached_arrays
            specs=[]
            if changed:
                try:
                    groups,_=scene_specs({},display,max_voxels=args.max_display_voxels,include_trees=False)
                    projection=occupancy_projection(display) if display is not None else None
                    specs=groups["map"]
                except (ValueError,KeyError,TypeError):
                    specs=[];projection=None
                grid=OccupancyGrid();grid.header.frame_id="map"
                grid.info.origin.orientation.w=1.
                grid.info.resolution=.25;grid.info.width=1;grid.info.height=1;grid.data=[-1]
                if projection is not None:
                    grid.info.resolution=projection["resolution"];grid.info.width=projection["width"];grid.info.height=projection["height"]
                    grid.info.origin.position.x,grid.info.origin.position.y,grid.info.origin.position.z=projection["origin"]
                    grid.data=projection["data"]
                self.cached_projection=grid
            stamp=self.get_clock().now().to_msg()
            heartbeat=time.monotonic()-self.last_published.get("map",0.)>=1.
            self.publish_specs("map",specs,stamp,changed,
                               epoch=state.get("epoch") if state is not None else None)
            if self.cached_projection is not None and (changed or heartbeat):
                self.cached_projection.header.stamp=stamp
                self.projection_pub.publish(self.cached_projection)
            self.map_signature=signature

        def tick(self):
            try:
                state,_,issues=read_runtime(args.latest_run,cache=self.state_cache,include_display=False)
                run_root=Path(state["_view_run_root"])
                if run_root!=self.active_run_root:
                    self.state_cache.clear()
                self.active_run_root=run_root
                self.active_robot_ids=[r["robot_id"] for r in state.get("robots") or []
                                       if isinstance(r.get("robot_id"),str) and re.fullmatch(r"[A-Za-z][A-Za-z0-9_]*",r["robot_id"])]
                try:
                    doc=self.state_cache.read(run_root/'shared/people.json')
                    if doc.get('scene_epoch')==state.get('epoch') and doc.get('scene_id')==state.get('scene_id'):
                        state['_people_document']=doc
                except (OSError,ValueError):pass
                # A shallow snapshot is immutable after assignment; slow
                # display callbacks can safely retain this version.
                self.state=state
                specs,transforms=scene_specs(state,None,issues=issues,include_trees=False)
                self.last_error=None
            except (OSError,ValueError,KeyError,TypeError) as error:
                message=str(error)
                if message!=self.last_error:
                    self.get_logger().warning("Runtime view waiting: "+message)
                    self.last_error=message
                self.state=None
                specs,transforms=scene_specs({},None,issues=["WAITING: "+message],include_trees=False)
            stamp=self.get_clock().now().to_msg()
            for kind in ("robots","paths","people"):
                self.publish_specs(kind,specs[kind],stamp,
                                   epoch=self.state.get("epoch") if self.state is not None else None)
            for spec in transforms:
                transform=TransformStamped();transform.header.frame_id="map";transform.header.stamp=stamp
                transform.child_frame_id=spec["child"]
                transform.transform.translation.x,transform.transform.translation.y,transform.transform.translation.z=spec["position"]
                transform.transform.rotation.z=math.sin(spec["yaw"]/2);transform.transform.rotation.w=math.cos(spec["yaw"]/2)
                self.tf.sendTransform(transform)

    rclpy.init(args=[])
    node=View()
    executor=MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__=="__main__":
    main()
