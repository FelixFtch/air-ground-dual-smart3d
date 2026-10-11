"""Owned test walkers. Ground truth drives scenario actors, never perception."""
import math
import carla

class People:
    DEFAULT_ROUTES=[[[-12,-6],[12,-6]],[[-12,2],[12,2]],[[-12,6],[12,6]],[[-6,-12],[-6,12]],[[5,-12],[5,12]],[[11,-12],[11,12]]]
    def __init__(self,world,count,speeds,routes=None):
        self.world=world;self.actors=[]
        bps=sorted(world.get_blueprint_library().filter('walker.pedestrian.*'),key=lambda x:x.id)
        # Each route is an arbitrary polyline of map-XY waypoints, walked in a
        # loop; a two-point route still behaves exactly like the old back-and-
        # forth segment. The scene profile's pedestrian_routes wins over the
        # defaults. Draw routes as UE splines and export them with
        # tools/ue_export_ped_routes.py to get this list without typing numbers.
        routes=routes or self.DEFAULT_ROUTES
        if not isinstance(routes,list) or not routes or len(routes)>64:
            raise ValueError('pedestrian_routes must be a nonempty list of at most 64 routes')
        cleaned=[]
        for seg in routes:
            if not isinstance(seg,list) or len(seg)<2 or len(seg)>64:raise ValueError('each route needs 2..64 [x,y] points')
            ends=[]
            for p in seg:
                if not isinstance(p,list) or len(p)!=2 or any(not isinstance(v,(int,float)) or not math.isfinite(v) for v in p):
                    raise ValueError('route points must be finite [x,y] numbers')
                ends.append((float(p[0]),float(p[1])))
            if any(math.dist(ends[i],ends[(i+1)%len(ends)])<1. for i in range(len(ends))):
                raise ValueError('route points must be at least 1 m apart')
            cleaned.append(ends)
        routes=cleaned
        for i in range(count):
            bp=bps[i%len(bps)];bp.set_attribute('role_name','ag_person_'+str(i))
            if bp.has_attribute('is_invincible'):bp.set_attribute('is_invincible','false')
            pts=list(routes[i%len(routes)])
            if i%2:pts=list(reversed(pts))
            actor=world.try_spawn_actor(bp,carla.Transform(carla.Location(pts[0][0],-pts[0][1],1.35)))
            if actor:self.actors.append({'actor':actor,'points':pts,'target':1,'speed':speeds[i%len(speeds)],'last':None,'stall':0})
    def update(self,dt):
        for s in self.actors:
            actor=s['actor']
            if not actor.is_alive:continue
            p=actor.get_location();q=s['points'][s['target']];dx,dy=q[0]-p.x,-q[1]-p.y;dist=math.hypot(dx,dy)
            current=(p.x,p.y)
            if s['last'] is not None and math.dist(current,s['last'])<.005:s['stall']+=dt
            else:s['stall']=0
            if dist<.65 or s['stall']>6:
                s['target']=(s['target']+1)%len(s['points']);s['stall']=0
            elif dist>0:
                actor.apply_control(carla.WalkerControl(carla.Vector3D(dx/dist,dy/dist,0),s['speed'],False))
            s['last']=current
    def close(self):
        for s in self.actors:
            if s['actor'].is_alive:s['actor'].destroy()
