from pyglet.window import key
import numpy as np
from graphics import camera
from resources import example_sprites

class Reference():

    def __init__(self, camera: camera.Camera, key_handler: key.KeyStateHandler, x=0, y=0, theta=0):
        self.sprite = example_sprites.reference(camera=camera)
        self.key_handler = key_handler
        
        self.speed = 2

    @property
    def pose(self): return self.sprite.pose
    @pose.setter
    def pose(self, value): self.sprite.pose = value

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
    
    def update(self, dt):
        if self.key_handler[key.RIGHT]:
            self.x += self.speed*dt
        if self.key_handler[key.LEFT]:
            self.x -= self.speed*dt
        if self.key_handler[key.UP]:
            self.y += self.speed*dt
        if self.key_handler[key.DOWN]:
            self.y -= self.speed*dt
        if self.key_handler[key.PERIOD]:
            self.theta += self.speed*dt
        if self.key_handler[key.COMMA]:
            self.theta -= self.speed*dt
