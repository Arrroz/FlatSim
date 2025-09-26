import numpy as np
from pyglet.window import mouse
from physics import body
from graphics import camera
from resources import example_sprites

class ExternalForce():

    def __init__(self, link_collection: body.BodyCollection, camera: camera.Camera, sprite_function=example_sprites.arrow, magnitude_mult=50):
        self.all_links = link_collection.movables
        self.camera = camera
        self.sprite_function = sprite_function
        self.magnitude_mult = magnitude_mult

        self.curr_link = None
        self.anchor = np.zeros((2,))

        self.sprite = None
        self.camera.push_handlers(self.on_mouse_press, self.on_mouse_release, self.on_mouse_drag)
        
        self.mouse = np.zeros((2,))
    
    def on_mouse_press(self, x, y, button, modifiers):
        if button != mouse.LEFT:
            return
        
        for l in self.all_links[::-1]:
            for s in l.sprite.shapes:
                if (x, y) in s:
                    self.curr_link = l

                    x, y = self._preprocess_coordinates(x, y)
                    p  = np.array([x, y])
                    self.mouse = p

                    p -= self.curr_link.pos
                    cos_theta = np.cos(self.curr_link.theta)
                    sin_theta = np.sin(self.curr_link.theta)
                    rot_mat = np.array([[cos_theta, sin_theta],
                                        [-sin_theta, cos_theta]])
                    self.anchor = rot_mat @ p
                    return

    def on_mouse_release(self, x, y, button, modifiers):
        if button != mouse.LEFT:
            return
        
        self.curr_link = None

    def on_mouse_drag(self, x, y, dx, dy, buttons, modifiers):
        if (not buttons & mouse.LEFT) or self.curr_link == None:
            return
        
        x, y = self._preprocess_coordinates(x, y)
        self.mouse = np.array([x, y])

    def _preprocess_coordinates(self, x, y):
        x = (x - self.camera.width/2) / self.camera.scale + self.camera.offset_x
        y = (y - self.camera.height/2) / self.camera.scale + self.camera.offset_y
        return (x, y)
    
    def update(self):
        if self.sprite != None:
            self.sprite.delete()
            self.sprite = None
        
        if self.curr_link == None:
            return

        cos_theta = np.cos(self.curr_link.theta)
        sin_theta = np.sin(self.curr_link.theta)
        rot_mat = np.array([[cos_theta, -sin_theta],
                            [sin_theta, cos_theta]])
        rotated_anchor = rot_mat @ self.anchor

        start = self.curr_link.pos + rotated_anchor
        t = self.mouse - start
        f = self.magnitude_mult * t

        self.curr_link.apply_force(f, rotated_anchor)

        self.sprite = self.sprite_function(self.camera, length=np.linalg.norm(t))
        self.sprite.pos = start
        self.sprite.theta = np.arctan2(t[1], t[0])
