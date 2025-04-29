import numpy as np

class Body():
    
    def __init__(self, mass=1, moi=1, restitution=0.6, movable=True):
        self.mass = mass
        self.moi = moi # moment of inertia
        self.restitution = restitution
        self.movable = movable

        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        
        self.vx = 0.0
        self.vy = 0.0
        self.w = 0.0
        
        self.rfx = 0.0
        self.rfy = 0.0
        self.rtorque = 0.0

    def apply_force(self, fx, fy, posx=0, posy=0):
        self.rfx += fx
        self.rfy += fy
        t = np.cross((posx, posy), (fx, fy))
        self.apply_torque(t)
    
    def apply_torque(self, t):
        self.rtorque += t


class BodyCollection():

    def __init__(self, bodies: list[Body]):
        self.bodies = bodies
        self.movables = [b for b in bodies if b.movable]
