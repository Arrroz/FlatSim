import numpy as np

class Body():

    def __init__(self, mass=1, moi=1, restitution=0.6, movable=True):
        self.mass = mass
        self.moi = moi # moment of inertia
        self.restitution = restitution
        self.movable = movable

        self.pose = np.zeros((3,))
        self.vel = np.zeros((3,))
        self.rwrench = np.zeros((3,))

        self.sprite_generator = None

    @property
    def pos(self): return self.pose[:2]
    @pos.setter
    def pos(self, value): self.pose[:2] = value

    @property
    def x(self): return self.pose[0]
    @x.setter
    def x(self, value): self.pose[0] = value
    
    @property
    def y(self): return self.pose[1]
    @y.setter
    def y(self, value): self.pose[1] = value
    
    @property
    def theta(self): return self.pose[2]
    @theta.setter
    def theta(self, value): self.pose[2] = value
    
    @property
    def lin_vel(self): return self.vel[:2]
    @lin_vel.setter
    def lin_vel(self, value): self.vel[:2] = value

    @property
    def vx(self): return self.vel[0]
    @vx.setter
    def vx(self, value): self.vel[0] = value
    
    @property
    def vy(self): return self.vel[1]
    @vy.setter
    def vy(self, value): self.vel[1] = value
    
    @property
    def w(self): return self.vel[2]
    @w.setter
    def w(self, value): self.vel[2] = value
    
    @property
    def rforce(self): return self.rwrench[:2]
    @rforce.setter
    def rforce(self, value): self.rwrench[:2] = value

    @property
    def rfx(self): return self.rwrench[0]
    @rfx.setter
    def rfx(self, value): self.rwrench[0] = value
    
    @property
    def rfy(self): return self.rwrench[1]
    @rfy.setter
    def rfy(self, value): self.rwrench[1] = value
    
    @property
    def rtorque(self): return self.rwrench[2]
    @rtorque.setter
    def rtorque(self, value): self.rwrench[2] = value
    
    
    def apply_force(self, f, pos=np.zeros((2,))):
        self.rforce += f
        t = np.cross(pos, f)
        self.apply_torque(t)
    
    def apply_torque(self, t):
        self.rtorque += t

