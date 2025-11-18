import numpy as np
from pyglet import shapes
from graphics.camera import Camera

class Sprite():

    def __init__(self, shapes: list[shapes.ShapeBase], camera: Camera = None, target = None):
        self.shapes = shapes
        self.target = target
        self.camera = camera

        self._pose = np.zeros((3,))
    
    @property
    def camera(self):
        return self._camera
    @camera.setter
    def camera(self, value: Camera):
        if value == None: return
        value.switch_to()

        if hasattr(self, '_camera') and self in self._camera.sprites:
            self._camera.sprites.remove(self)
        self._camera = value
        value.sprites.append(self)

        for shape in self.shapes:
            shape.batch = value.batch

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

    def delete(self):
        self.camera.sprites.remove(self)
        for s in self.shapes:
            s.delete()
