from pyglet import window, graphics, gl, shapes
import numpy as np
from graphics import shapes_sprite

class Camera(window.Window):

    def __init__(self, scale=100, offset_x=0, offset_y=1, *args, **kwargs):
        super().__init__(*args, **kwargs)
        
        self.sprites = [] # type: list[shapes_sprite.Sprite]
        self.batch = graphics.Batch()

        self.scale = scale
        self.offset_x = offset_x
        self.offset_y = offset_y

        gl.glClearColor(.8, .8, .8, 1)
        self.set_location(100, 130)
    
    @property
    def batch(self): return self._batch
    @batch.setter
    def batch(self, value: graphics.Batch):
        self.switch_to()
        for sprite in self.sprites:
            for shape in sprite.shapes:
                shape.batch = value
        self._batch = value
    
    @property
    def scale(self): return self._scale
    @scale.setter
    def scale(self, value):
        for sprite in self.sprites:
            for shape in sprite.shapes:
                shape.anchor_x *= value/self.scale
                shape.anchor_y *= value/self.scale
                if shape.__class__ == shapes.Circle:
                    shape.radius *= value/self.scale
                elif shape.__class__ == shapes.Rectangle:
                    shape.width *= value/self.scale
                    shape.height *= value/self.scale
        self._scale = value
    
    def add_sprite(self, sprite: shapes_sprite.Sprite):
        self.sprites.append(sprite)
        
        self.switch_to()
        for s in sprite.shapes:
            s.batch = self.batch
    
    def del_sprite(self, sprite: shapes_sprite.Sprite):
        self.sprites.remove(sprite)

        for s in sprite.shapes:
            s.delete()

    def on_mouse_drag(self, x, y, dx, dy, buttons, modifiers):
        if buttons & window.mouse.RIGHT:
            self.offset_x -= dx / self.scale
            self.offset_y -= dy / self.scale
    
    def on_mouse_scroll(self, x, y, scroll_x, scroll_y):
        self.scale += 0.1 * scroll_y * self.scale
    
    def on_draw(self):
        self.switch_to()
        self.clear()

        # updating each shape's coordinates every iteration instead of when the sprite's coordinates change or when the camera moves is more expensive but much simpler
        for sprite in self.sprites:
            for shape in sprite.shapes:
                shape.x = (sprite.x - self.offset_x) * self.scale + self.width/2
                shape.y = (sprite.y - self.offset_y) * self.scale + self.height/2
                shape.rotation = -np.rad2deg(sprite.theta)

        self.batch.draw()
