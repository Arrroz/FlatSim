import numpy as np
from pyglet import shapes
from graphics.camera import Camera

class Sprite():

    def __init__(self, shapes: list[shapes.ShapeBase], body = None, camera: Camera = None):
        self.shapes = shapes
        self.body = body
        self.camera = camera

        if body != None and not (self in body.sprites):
            body.sprites.append(self)

        self._pose = np.zeros((3,))
    
    @property
    def camera(self):
        return self._camera
    @camera.setter
    def camera(self, value: Camera):
        if hasattr(self, '_camera') and self in self._camera.sprites:
            self._camera.sprites.remove(self)
        self._camera = value
        value.sprites.append(self)

        for shape in self.shapes:
            shape.batch = self._camera.batch

    @property
    def pose(self):
        if self.body == None: return self._pose
        else: return self.body.pose
    @pose.setter
    def pose(self, value):
        if self.body == None: self._pose = value
        else: self.body.pose = value

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
