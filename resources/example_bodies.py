from physics import collision
from graphics import camera
from resources import example_collidable_features, example_sprites

def circle(camera: camera.Camera, radius=0.3, mass=1, moi=None):
    collidable_features = example_collidable_features.circle(radius=radius)
    sprite = example_sprites.circle(camera=camera, radius=radius)
    if moi == None: moi = mass * radius * radius / 2 # from online list of moments of inertia
    return collision.Collidable(collidable_features=collidable_features, mass=mass, moi=moi, sprites=[sprite])

def rectangle(camera: camera.Camera, width=2, height=0.6, mass=1, moi=None):
    collidable_features = example_collidable_features.rectangle(anchor_x=-width/2, anchor_y=-height/2, width=width, height=height)
    sprite = example_sprites.rectangle(camera=camera, width=width, height=height)
    if moi == None: moi = mass * (height*height + width*width) / 12 # from online list of moments of inertia
    return collision.Collidable(collidable_features=collidable_features, mass=mass, moi=moi, sprites=[sprite])

def wheel(camera: camera.Camera, radius=0.3, mass=1, moi=None):
    collidable_features = example_collidable_features.circle(radius=radius)
    sprite = example_sprites.wheel(camera=camera, radius=radius)
    if moi == None: moi = mass * radius * radius / 2 # from online list of moments of inertia
    return collision.Collidable(collidable_features=collidable_features, mass=mass, moi=moi, sprites=[sprite])

def robot_body(camera: camera.Camera, width=2, height=0.6, mass=1, moi=None):
    collidable_features = example_collidable_features.rectangle(anchor_x=-width/2, anchor_y=-height/2, width=width, height=height)
    sprite = example_sprites.robot_body(camera=camera, width=width, height=height)
    if moi == None: moi = mass * (height*height + width*width) / 12 # from online list of moments of inertia
    return collision.Collidable(collidable_features=collidable_features, mass=mass, moi=moi, sprites=[sprite])

def leg_link(camera: camera.Camera, length=0.8, width=0.2, mass=1, moi=None):
    collidable_features = example_collidable_features.leg_link(length=length, width=width)
    sprite = example_sprites.leg_link(camera=camera, length=length, width=width)
    if moi == None: moi = mass * (length*length + width*width) / 12 # from online list of moments of inertia
    return collision.Collidable(collidable_features=collidable_features, mass=mass, moi=moi, sprites=[sprite])

def ground(camera: camera.Camera, width=100, height=50):
    collidable_features = example_collidable_features.rectangle(anchor_x=-width/2, anchor_y=-height/2, width=width, height=height)
    sprite = example_sprites.rectangle(camera=camera, width=width, height=height, color=(100, 100, 100))
    l = collision.Collidable(collidable_features=collidable_features, movable=False, sprites=[sprite])
    l.y = -1.5 - height/2
    return l
