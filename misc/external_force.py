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
        self.anchor_x = 0
        self.anchor_y = 0

        self.sprite = None
        self.camera.push_handlers(self.on_mouse_press, self.on_mouse_release, self.on_mouse_drag)
        
        self.mouse_x = 0
        self.mouse_y = 0
    
    def on_mouse_press(self, x, y, button, modifiers):
        if button != mouse.LEFT:
            return
        
        for l in self.all_links[::-1]:
            for s in l.sprite.shapes:
                if (x, y) in s:
                    self.curr_link = l

                    x, y = self._preprocess_coordinates(x, y)
                    self.mouse_x = x
                    self.mouse_y = y

                    x -= l.x
                    y -= l.y
                    cos_theta = np.cos(l.theta)
                    sin_theta = np.sin(l.theta)
                    self.anchor_x = x*cos_theta + y*sin_theta
                    self.anchor_y = -x*sin_theta + y*cos_theta
                    return

    def on_mouse_release(self, x, y, button, modifiers):
        if button != mouse.LEFT:
            return
        
        self.curr_link = None

    def on_mouse_drag(self, x, y, dx, dy, buttons, modifiers):
        if (not buttons & mouse.LEFT) or self.curr_link == None:
            return
        
        self.mouse_x, self.mouse_y = self._preprocess_coordinates(x, y)

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
        rotated_anchor_x = self.anchor_x*cos_theta - self.anchor_y*sin_theta
        rotated_anchor_y = self.anchor_x*sin_theta + self.anchor_y*cos_theta
        
        start_x = self.curr_link.x + rotated_anchor_x
        start_y = self.curr_link.y + rotated_anchor_y
        tx = self.mouse_x - start_x
        ty = self.mouse_y - start_y
        fx = self.magnitude_mult * tx
        fy = self.magnitude_mult * ty

        self.curr_link.apply_force(fx, fy, rotated_anchor_x, rotated_anchor_y)

        self.sprite = self.sprite_function(self.camera, length=np.sqrt(tx*tx + ty*ty))
        self.sprite.x = start_x
        self.sprite.y = start_y
        self.sprite.theta = np.arctan2(ty, tx)
