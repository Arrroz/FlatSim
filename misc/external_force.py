import numpy as np
from pyglet.window import mouse
from physics import body
from graphics import camera
from resources import example_sprites

class ExternalForce():

    def __init__(self, camera: camera.Camera, sprite_function=example_sprites.arrow, magnitude_mult=50):
        self.camera = camera
        self.sprite_function = sprite_function
        self.magnitude_mult = magnitude_mult

        self.curr_body = None
        self.anchor = np.zeros((2,))

        self.sprite = None
        self.camera.push_handlers(self.on_mouse_press, self.on_mouse_release, self.on_mouse_drag)
        
        self.mouse = np.zeros((2,))
    
    def on_mouse_press(self, x, y, button, modifiers):
        if button != mouse.LEFT:
            return
        
        for sprite in self.camera.sprites:
            if not isinstance(sprite.target, body.Body): continue
            if not sprite.target.movable: continue

            for shape in sprite.shapes:
                if (x, y) in shape:
                    self.curr_body = sprite.target

                    x, y = self._preprocess_coordinates(x, y)
                    p  = np.array([x, y])
                    self.mouse = p

                    p -= self.curr_body.pos
                    cos_theta = np.cos(self.curr_body.theta)
                    sin_theta = np.sin(self.curr_body.theta)
                    rot_mat = np.array([[cos_theta, sin_theta],
                                        [-sin_theta, cos_theta]])
                    self.anchor = rot_mat @ p
                    return

    def on_mouse_release(self, x, y, button, modifiers):
        if button != mouse.LEFT:
            return
        
        self.curr_body = None

    def on_mouse_drag(self, x, y, dx, dy, buttons, modifiers):
        if (not buttons & mouse.LEFT) or self.curr_body == None:
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
        
        if self.curr_body == None:
            return

        cos_theta = np.cos(self.curr_body.theta)
        sin_theta = np.sin(self.curr_body.theta)
        rot_mat = np.array([[cos_theta, -sin_theta],
                            [sin_theta, cos_theta]])
        rotated_anchor = rot_mat @ self.anchor

        start = self.curr_body.pos + rotated_anchor
        t = self.mouse - start
        f = self.magnitude_mult * t

        self.curr_body.apply_force(f, rotated_anchor)

        self.sprite = self.sprite_function(self.camera, length=np.linalg.norm(t))
        self.sprite.pos = start
        self.sprite.theta = np.arctan2(t[1], t[0])
