"""Owned test walkers. Ground truth drives scenario actors, never perception."""
import math
import carla

class People:
    def __init__(self,world,count,speeds):
        self.world=world;self.actors=[]
        bps=sorted(world.get_blueprint_library().filter('walker.pedestrian.*'),key=lambda x:x.id)
        routes=[((-12,-6),(12,-6)),((-12,2),(12,2)),((-12,6),(12,6)),((-6,-12),(-6,12)),((5,-12),(5,12)),((11,-12),(11,12))]
        for i in range(count):
            bp=bps[i%len(bps)];bp.set_attribute('role_name','ag_person_'+str(i))
            if bp.has_attribute('is_invincible'):bp.set_attribute('is_invincible','false')
            a,b=routes[i%len(routes)];a,b=(b,a) if i%2 else (a,b)
            actor=world.try_spawn_actor(bp,carla.Transform(carla.Location(a[0],-a[1],1.35)))
            if actor:self.actors.append({'actor':actor,'ends':[a,b],'target':1,'speed':speeds[i%len(speeds)],'last':None,'stall':0})
    def update(self,dt):
        for s in self.actors:
            actor=s['actor']
            if not actor.is_alive:continue
            p=actor.get_location();q=s['ends'][s['target']];dx,dy=q[0]-p.x,-q[1]-p.y;dist=math.hypot(dx,dy)
            current=(p.x,p.y)
            if s['last'] is not None and math.dist(current,s['last'])<.005:s['stall']+=dt
            else:s['stall']=0
            if dist<.65 or s['stall']>6:
                s['target']=1-s['target'];s['stall']=0
            elif dist>0:
                actor.apply_control(carla.WalkerControl(carla.Vector3D(dx/dist,dy/dist,0),s['speed'],False))
            s['last']=current
    def close(self):
        for s in self.actors:
            if s['actor'].is_alive:s['actor'].destroy()
