from physics import collision
from graphics import shapes_sprite

class Link(collision.Collidable):

    def __init__(self, sprite: shapes_sprite.Sprite, *args, **kwargs):
        self.sprite = sprite

        super().__init__(*args, **kwargs)

    @property
    def pose(self): return self.sprite.pose
    @pose.setter
    def pose(self, value): self.sprite.pose = value
