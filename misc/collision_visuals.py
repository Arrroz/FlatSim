import numpy as np
from physics import collision
from graphics import camera
from resources import example_sprites

class CollisionVisuals():

    def __init__(self, camera: camera.Camera, sprite_function=example_sprites.collision):
        self.camera = camera
        self.sprite_function = sprite_function

        self.sprites = []
    
    def update(self, collisions: list[collision.Collision]):
        for s in self.sprites:
            s.delete()
        self.sprites = []

        for c in collisions:
            s1 = self.sprite_function(camera=self.camera)
            s1.x = c.collidable1.x + c.relative_pos1[0]
            s1.y = c.collidable1.y + c.relative_pos1[1]
            s1.theta = np.arctan2(c.normal[1], c.normal[0])
            self.sprites.append(s1)

            s2 = self.sprite_function(camera=self.camera)
            s2.x = c.collidable2.x + c.relative_pos2[0]
            s2.y = c.collidable2.y + c.relative_pos2[1]
            s2.theta = np.arctan2(-c.normal[1], -c.normal[0])
            self.sprites.append(s2)
