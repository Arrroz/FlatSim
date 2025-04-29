from physics import collision
from graphics import shapes_sprite

class Link(collision.Collidable):

    def __init__(self, sprite: shapes_sprite.Sprite, *args, **kwargs):
        self.sprite = sprite

        super().__init__(*args, **kwargs)

    def __setattr__(self, name, value):
        if name == 'x' or name == 'y' or name == 'theta':
            self.sprite.__setattr__(name, value)
                
        super().__setattr__(name, value)

