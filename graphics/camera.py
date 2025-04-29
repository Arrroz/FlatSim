from pyglet import window, graphics, gl

class Camera(window.Window):

    def __init__(self, scale=100, offset_x=0, offset_y=0, *args, **kwargs):
        self.sprites = []
        super().__init__(*args, **kwargs)
        self.batch = graphics.Batch()
        self.scale = scale
        self.offset_x = offset_x
        self.offset_y = offset_y

        gl.glClearColor(.8, .8, .8, 1)
        self.set_location(100, 130)

    def __setattr__(self, name: str, value):
        if name == 'batch':
            for s in self.sprites:
                s._update_batch(value)
        elif name == 'scale':
            for s in self.sprites:
                s._update_scale(value)

        super().__setattr__(name, value)

        for s in self.sprites:
            s.x = s.x
            s.y = s.y
        
    def on_mouse_drag(self, x, y, dx, dy, buttons, modifiers):
        if buttons & window.mouse.RIGHT:
            self.offset_x -= dx / self.scale
            self.offset_y -= dy / self.scale
    
    def on_mouse_scroll(self, x, y, scroll_x, scroll_y):
        self.scale += 10 * scroll_y
    
    def on_draw(self):
        self.clear()
        self.batch.draw()
