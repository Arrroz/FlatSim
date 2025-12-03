from physics import collision
from resources import example_collidable_features, example_sprites

def circle(radius=0.3, mass=1, moi=None):
    collidable_features = example_collidable_features.circle(radius=radius)
    if moi == None: moi = mass * radius * radius / 2 # from online list of moments of inertia
    
    body = collision.Collidable(collidable_features=collidable_features, mass=mass, moi=moi)
    body.sprite_generator = lambda: example_sprites.circle(radius=radius, target=body)
    
    return body

def rectangle(width=2, height=0.6, mass=1, moi=None):
    collidable_features = example_collidable_features.rectangle(anchor_x=-width/2, anchor_y=-height/2, width=width, height=height)
    if moi == None: moi = mass * (height*height + width*width) / 12 # from online list of moments of inertia
    
    body = collision.Collidable(collidable_features=collidable_features, mass=mass, moi=moi)
    body.sprite_generator = lambda: example_sprites.rectangle(width=width, height=height, target=body)
    
    return body

def wheel(radius=0.3, mass=1, moi=None):
    collidable_features = example_collidable_features.circle(radius=radius)
    if moi == None: moi = mass * radius * radius / 2 # from online list of moments of inertia
    
    body = collision.Collidable(collidable_features=collidable_features, mass=mass, moi=moi)
    body.sprite_generator = lambda: example_sprites.wheel(radius=radius, target=body)
    
    return body

def robot_body(width=2, height=0.6, mass=1, moi=None):
    collidable_features = example_collidable_features.rectangle(anchor_x=-width/2, anchor_y=-height/2, width=width, height=height)
    if moi == None: moi = mass * (height*height + width*width) / 12 # from online list of moments of inertia
    
    body = collision.Collidable(collidable_features=collidable_features, mass=mass, moi=moi)
    body.sprite_generator = lambda: example_sprites.robot_body(width=width, height=height, target=body)
    
    return body

def leg_link(length=0.8, width=0.2, mass=1, moi=None):
    collidable_features = example_collidable_features.leg_link(length=length, width=width)
    if moi == None: moi = mass * (length*length + width*width) / 12 # from online list of moments of inertia
    
    body = collision.Collidable(collidable_features=collidable_features, mass=mass, moi=moi)
    body.sprite_generator = lambda: example_sprites.leg_link(length=length, width=width, target=body)
    
    return body

def ground(width=100, height=50):
    collidable_features = example_collidable_features.rectangle(anchor_x=-width/2, anchor_y=-height/2, width=width, height=height)
    
    body = collision.Collidable(collidable_features=collidable_features, movable=False)
    body.y = -height/2
    body.sprite_generator = lambda: example_sprites.rectangle(width=width, height=height, color=(100, 100, 100), target=body)
    
    return body
