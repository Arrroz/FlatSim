from pyglet import window, graphics, gl
import numpy as np

class Camera(window.Window):

    def __init__(self, scale=100, offset_x=0, offset_y=0, *args, **kwargs):
        from graphics.shapes_sprite import Sprite
        self.sprites = [] # type: list[Sprite]
        super().__init__(*args, **kwargs)
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
        for s in self.sprites:
            s._update_batch(value)
        self._batch = value
    
    @property
    def scale(self): return self._scale
    @scale.setter
    def scale(self, value):
        for s in self.sprites:
            s._update_scale(value)
        self._scale = value

    def on_mouse_drag(self, x, y, dx, dy, buttons, modifiers):
        if buttons & window.mouse.RIGHT:
            self.offset_x -= dx / self.scale
            self.offset_y -= dy / self.scale
    
    def on_mouse_scroll(self, x, y, scroll_x, scroll_y):
        self.scale += 0.1 * scroll_y * self.scale
    
    def on_draw(self):
        self.clear()

        # updating each shape's coordinates every iteration instead of when the sprite's coordinates change or when the camera moves is more expensive but much simpler
        for sprite in self.sprites:
            for shape in sprite.shapes:
                shape.x = (sprite.x - self.offset_x) * self.scale + self.width/2
                shape.y = (sprite.y - self.offset_y) * self.scale + self.height/2
                shape.rotation = -np.rad2deg(sprite.theta)

        self.batch.draw()
