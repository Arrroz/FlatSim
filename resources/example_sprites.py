from pyglet import shapes
from graphics import camera, shapes_sprite

def leg_link(camera: camera.Camera, length=0.8, width=0.2):
    rectangle_length = length * camera.scale
    rectangle_width = (width-0.05) * camera.scale
    circle_radius = (width/2) * camera.scale

    rectangle = shapes.Rectangle(0, 0, rectangle_length, rectangle_width, color=(150, 0, 0))
    circle1 = shapes.Circle(0, 0, circle_radius, color=(100, 0, 200))
    circle2 = shapes.Circle(0, 0, circle_radius, color=(100, 0, 200))

    rectangle.anchor_position = (rectangle_length/2, rectangle_width/2)
    circle1.anchor_position = (rectangle_length/2, 0)
    circle2.anchor_position = (-rectangle_length/2, 0)

    return shapes_sprite.Sprite([rectangle, circle1, circle2], camera=camera)

def rectangle(camera: camera.Camera, width=2, height=0.6, color=(50, 100, 150)):
    width *= camera.scale
    height *= camera.scale

    rectangle = shapes.Rectangle(0, 0, width, height, color=color)
    rectangle.anchor_position = (width/2, height/2)
    
    return shapes_sprite.Sprite([rectangle], camera=camera)

def robot_body(camera: camera.Camera, width=2, height=0.6, color=(50, 100, 150)):
    width *= camera.scale
    height *= camera.scale

    rectangle = shapes.Rectangle(0, 0, width, height, color=color)
    dot = shapes.Circle(0, 0, 5, color=(150, 0, 0))
    dash = shapes.Rectangle(0, 0, 10, 4, color=(150, 0, 0))

    rectangle.anchor_position = (width/2, height/2)
    dash.anchor_position = (0, 2)

    return shapes_sprite.Sprite([rectangle, dot, dash], camera=camera)

def circle(camera: camera.Camera, radius=0.3):
    radius *= camera.scale
    c = shapes.Circle(0, 0, radius, color=(100, 0, 200))
    c.anchor_position = (0, 0)
    return shapes_sprite.Sprite([c], camera=camera)

def wheel(camera: camera.Camera, radius=0.3):
    radius *= camera.scale
    line_length = 1.5*radius
    line_width = 0.1*radius
    
    circle = shapes.Circle(0, 0, radius, color=(100, 0, 200))
    line1 = shapes.Rectangle(0, 0, line_width, line_length, color=(150, 0, 0))
    line2 = shapes.Rectangle(0, 0, line_length, line_width, color=(150, 0, 0))

    circle.anchor_position = (0, 0)
    line1.anchor_position = (line_width/2, line_length/2)
    line2.anchor_position = (line_length/2, line_width/2)

    return shapes_sprite.Sprite([circle, line1, line2], camera=camera)

def arrow(camera: camera.Camera, length=1):
    triangle_base = 20
    triangle_height = 20
    rectangle_length = max(length*camera.scale-triangle_height, 0)
    rectangle_width = triangle_base * 0.3

    rectangle = shapes.Rectangle(0, 0, rectangle_length, rectangle_width,
                                 color=(50, 200, 50))
    triangle = shapes.Triangle(0, 0, 0, triangle_base, triangle_height, triangle_base/2, 
                               color=(50, 200, 50))

    rectangle.anchor_position = (0, rectangle_width/2)
    triangle.anchor_position = (-rectangle_length, triangle_base/2)

    return shapes_sprite.Sprite([rectangle, triangle], camera=camera)

def point(camera: camera.Camera):
    c = shapes.Circle(0, 0, 5, color=(200, 0, 0))
    return shapes_sprite.Sprite([c], camera=camera)

def collision(camera: camera.Camera):
    n = shapes.Rectangle(0, 0, 30, 4, color=(0, 150, 0))
    p = shapes.Circle(0, 0, 5, color=(200, 0, 0))
    
    n.anchor_position = (0, 2)

    return shapes_sprite.Sprite([n, p], camera=camera)

def reference(camera: camera.Camera):
    p = shapes.Circle(0, 0, 5, color=(0, 200, 0))
    a = shapes.Rectangle(0, 0, 10, 4, color=(0, 200, 0))

    a.anchor_position = (0, 2)

    return shapes_sprite.Sprite([p, a], camera=camera)
