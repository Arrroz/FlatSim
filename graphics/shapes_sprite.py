import numpy as np
from pyglet import shapes

class Sprite():

    def __init__(self, shapes: list[shapes.ShapeBase], camera=None, target=None):
        self.shapes = shapes
        self.target = target

        if camera != None:
            camera.add_sprite(self)

        self._pose = np.zeros((3,))
    
    @property
    def pose(self):
        if self.target == None: return self._pose
        else: return self.target.pose
    @pose.setter
    def pose(self, value):
        if self.target == None: self._pose = value
        else: self.target.pose = value

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
