import numpy as np
from pyglet import shapes
from graphics.camera import Camera

class Sprite():

    def __init__(self, shapes: list[shapes.ShapeBase], camera: Camera):
        self.shapes = shapes
        self.camera = camera

        self.pose = np.zeros((3,))
    
    @property
    def camera(self):
        return self._camera
    @camera.setter
    def camera(self, value: Camera):
        self._camera = value
        self._update_batch(value.batch)
        if self in self._camera.sprites:
            self._camera.sprites.remove(self)
        value.sprites.append(self)

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

    def _update_batch(self, batch):
        for s in self.shapes:
            s.batch = batch
    
    def _update_scale(self, scale):
        for s in self.shapes:
            s.anchor_x *= scale/self.camera.scale
            s.anchor_y *= scale/self.camera.scale
            if s.__class__ == shapes.Circle:
                s.radius *= scale/self.camera.scale
            elif s.__class__ == shapes.Rectangle:
                s.width *= scale/self.camera.scale
                s.height *= scale/self.camera.scale
    
    def delete(self):
        self.camera.sprites.remove(self)
        for s in self.shapes:
            s.delete()
