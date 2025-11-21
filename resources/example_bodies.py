from physics import collision
from graphics import camera
from resources import example_collidable_features, example_sprites

def circle(radius=0.3, mass=1, moi=None, cameras: list[camera.Camera] = []):
    collidable_features = example_collidable_features.circle(radius=radius)
    if moi == None: moi = mass * radius * radius / 2 # from online list of moments of inertia
    body = collision.Collidable(collidable_features=collidable_features, mass=mass, moi=moi)
    
    for cam in cameras:
        example_sprites.circle(camera=cam, target=body, radius=radius)
    
    return body

def rectangle(width=2, height=0.6, mass=1, moi=None, cameras: list[camera.Camera] = []):
    collidable_features = example_collidable_features.rectangle(anchor_x=-width/2, anchor_y=-height/2, width=width, height=height)
    if moi == None: moi = mass * (height*height + width*width) / 12 # from online list of moments of inertia
    body = collision.Collidable(collidable_features=collidable_features, mass=mass, moi=moi)
    
    for cam in cameras:
        example_sprites.rectangle(camera=cam, target=body, width=width, height=height)
    
    return body

def wheel(radius=0.3, mass=1, moi=None, cameras: list[camera.Camera] = []):
    collidable_features = example_collidable_features.circle(radius=radius)
    if moi == None: moi = mass * radius * radius / 2 # from online list of moments of inertia
    body = collision.Collidable(collidable_features=collidable_features, mass=mass, moi=moi)
    
    for cam in cameras:
        example_sprites.wheel(camera=cam, target=body, radius=radius)
    
    return body

def robot_body(width=2, height=0.6, mass=1, moi=None, cameras: list[camera.Camera] = []):
    collidable_features = example_collidable_features.rectangle(anchor_x=-width/2, anchor_y=-height/2, width=width, height=height)
    if moi == None: moi = mass * (height*height + width*width) / 12 # from online list of moments of inertia
    body = collision.Collidable(collidable_features=collidable_features, mass=mass, moi=moi)
    
    for cam in cameras:
        example_sprites.robot_body(camera=cam, target=body, width=width, height=height)
    
    return body

def leg_link(length=0.8, width=0.2, mass=1, moi=None, cameras: list[camera.Camera] = []):
    collidable_features = example_collidable_features.leg_link(length=length, width=width)
    if moi == None: moi = mass * (length*length + width*width) / 12 # from online list of moments of inertia
    body = collision.Collidable(collidable_features=collidable_features, mass=mass, moi=moi)
    
    for cam in cameras:
        example_sprites.leg_link(camera=cam, target=body, length=length, width=width)
    
    return body

def ground(width=100, height=50, cameras: list[camera.Camera] = []):
    collidable_features = example_collidable_features.rectangle(anchor_x=-width/2, anchor_y=-height/2, width=width, height=height)
    body = collision.Collidable(collidable_features=collidable_features, movable=False)
    body.y = -height/2
    
    for cam in cameras:
        example_sprites.rectangle(camera=cam, target=body, width=width, height=height, color=(100, 100, 100))
    
    return body
