import numpy as np
from pyglet import shapes
from graphics import camera

class Sprite():

    def __init__(self, shapes: list[shapes.ShapeBase], camera: camera.Camera):
        self.shapes = shapes
        self.camera = camera

        self.x = 0
        self.y = 0
        self.theta = 0

    def __setattr__(self, name: str, value):
        self.__dict__[name] = value

        if name == 'x':
            for s in self.shapes:
                s.__setattr__(name, (value - self.camera.offset_x) * self.camera.scale + self.camera.width/2)
        elif name == 'y':
            for s in self.shapes:
                s.__setattr__(name, (value - self.camera.offset_y) * self.camera.scale + self.camera.height/2)
        elif name == 'theta':
            for s in self.shapes:
                s.rotation = -np.rad2deg(value)
        elif name == 'camera':
            self._update_batch(value.batch)
            self._update_scale(value.scale) # TODO: how does this work? it doesn't xP
            if self in self.camera.sprites:
                self.camera.sprites.remove(self)
            value.sprites.append(self)
    
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
