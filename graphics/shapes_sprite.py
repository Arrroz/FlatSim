import numpy as np
from pyglet import shapes

class Sprite():

    def __init__(self, shapes: list[shapes.ShapeBase], camera=None, target=None):
        self.shapes = shapes
        self.target = target

        self._pose = np.zeros((3,))
        self._scale = 1

        if camera != None:
            camera.add_sprite(self)
    
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

    @property
    def scale(self):
        return self._scale
    @scale.setter
    def scale(self, value):
        for shape in self.shapes:
            shape.anchor_x *= value/self._scale
            shape.anchor_y *= value/self._scale
            if isinstance(shape, shapes.Circle):
                shape.radius *= value/self._scale
            elif isinstance(shape, shapes.Rectangle):
                shape.width *= value/self._scale
                shape.height *= value/self._scale
            elif isinstance(shape, shapes.Triangle):
                shape.x *= value/self._scale
                shape.y *= value/self._scale
                shape.x2 *= value/self._scale
                shape.y2 *= value/self._scale
                shape.x3 *= value/self._scale
                shape.y3 *= value/self._scale
        self._scale = value
