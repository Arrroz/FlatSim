from physics import collision

def circle(anchor_x=0, anchor_y=0, radius=0.3):
    return [collision.Circle(anchor_x, anchor_y, radius)]

def polygon(points: list[(float, float)]):
    lines = []

    curr_p = points[0]
    for next_p in points[1:]:
        lines.append(collision.Line(curr_p[0], curr_p[1], next_p[0], next_p[1]))
        curr_p = next_p
    next_p = points[0]
    lines.append(collision.Line(curr_p[0], curr_p[1], next_p[0], next_p[1]))

    return lines

def rectangle(anchor_x=0, anchor_y = 0, width=2, height=0.6):
    points = [(anchor_x,        height+anchor_y),
              (width+anchor_x,  height+anchor_y),
              (width+anchor_x,  anchor_y),
              (anchor_x,        anchor_y)]
    
    return polygon(points)

def leg_link(length=0.8, width=0.2):
    rectangle_length = length
    rectangle_width = width-0.05
    circle_radius = width/2

    circle1_anchor_x = rectangle_length/2
    circle1_anchor_y = 0
    circle2_anchor_x = -rectangle_length/2
    circle2_anchor_y = 0

    line1 = collision.Line(-rectangle_length/2, rectangle_width/2, rectangle_length/2, rectangle_width/2)
    line2 = collision.Line(-rectangle_length/2, -rectangle_width/2, rectangle_length/2, -rectangle_width/2)
    circle1 = collision.Circle(anchor_x=circle1_anchor_x, anchor_y=circle1_anchor_y, radius=circle_radius)
    circle2 = collision.Circle(anchor_x=circle2_anchor_x, anchor_y=circle2_anchor_y, radius=circle_radius)
    
    return [line1, line2, circle1, circle2]
