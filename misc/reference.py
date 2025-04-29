from pyglet.window import key
from graphics import camera
from resources import example_sprites

class Reference():

    def __init__(self, camera: camera.Camera, key_handler: key.KeyStateHandler, x=0, y=0, theta=0):
        self.sprite = example_sprites.reference(camera=camera)
        self.key_handler = key_handler
        
        self.x = x
        self.y = y
        self.theta = theta

        self.speed = 2
    
    def __setattr__(self, name: str, value):
        super().__setattr__(name, value)

        if name == "x":
            self.sprite.x = value
        elif name == "y":
            self.sprite.y = value
        elif name == "theta":
            self.sprite.theta = value

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
