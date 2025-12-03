import pyglet
import numpy as np
from physics import engine, utils
from graphics import camera
from resources import example_bodies, example_robots
from misc import collision_visuals, external_force, reference
from system import joint

shapes_debug = False # debugging with random shapes; TODO: remove once done
rolling_contact_joint = False # TODO: remove once unnecessary

# numpy setup
np.set_printoptions(precision=2, suppress=True)

# window
cam = camera.Camera(width=800, height=600)

key_handler = pyglet.window.key.KeyStateHandler()
cam.push_handlers(key_handler)

# setup bodies and constraints (constraints are in the test_robot)
ground = example_bodies.ground()

if not shapes_debug:
    test_robot, robot_controller = example_robots.biped()

    if rolling_contact_joint:
        ground_joint = joint.RollingContactJoint(radius=0.2, normal=np.array([0,1]), # TODO: wheel radius and ground anchor are hard-coded
                                                parent=ground, anchor_parent=np.array([0, 25]),
                                                child=test_robot.joints[-1].child, anchor_child=np.array([0, 0]),
                                                actuated=False)

if shapes_debug:
    circle1 = example_bodies.wheel()
    circle1.x = 2
    circle1.y = 1
    circle2 = example_bodies.wheel(radius=0.5)
    circle2.x = -2
    circle2.y = 1
    rect1 = example_bodies.rectangle()
    rect1.y = 2
    rect2 = example_bodies.rectangle()
    rect2.y = 1

# engine stuff
if shapes_debug:
    # bodies = [rect1, ground]
    bodies = [circle1, rect1, circle2, rect2, ground]
    constraints = []
if not shapes_debug:
    bodies = test_robot.links() + [ground]
    constraints = test_robot.joints

    if rolling_contact_joint:
        constraints.append(ground_joint)

eng = engine.Engine(bodies, constraints)

if not shapes_debug:
    if rolling_contact_joint:
        eng.collision_handler.add_collision_ignore_set(test_robot.links() + [ground])
    else:
        eng.collision_handler.add_collision_ignore_set(test_robot.links())

# miscellaneous
ext_force = external_force.ExternalForce(camera=cam)
col_visuals = collision_visuals.CollisionVisuals(camera=cam)
ref = reference.Reference(key_handler=key_handler)

# create sprites
for b in bodies:
    cam.add_sprite(b.sprite_generator())
cam.add_sprite(ref.sprite_generator())

if not shapes_debug:
    ref.x = test_robot.base.x
    ref.y = test_robot.base.y
    ref.theta = test_robot.base.theta

# data recording
if not shapes_debug:
    t = 0
    data = np.empty(shape=(0,7))

# sim control
controller_active = False
play = False

# loop
def update(dt):
    # apply forces
    global play
    if key_handler[pyglet.window.key.Q]:
        play = True
    if key_handler[pyglet.window.key.W]:
        play = False

    global controller_active
    if key_handler[pyglet.window.key.Z]:
        controller_active = True
    if key_handler[pyglet.window.key.X]:
        controller_active = False
    
    #misc
    ref.update(dt)
    
    if not play:
        return

    if not shapes_debug:
        if controller_active:
            robot_controller.update(dt, ref.pose)

    [b.apply_force(utils.gravity * b.mass) for b in bodies if b.movable] # gravity
    ext_force.update() # external force

    # step forward the simulation
    eng.step(dt)

    # data recording
    if not shapes_debug:
        global data
        global t
        # x, y, theta, x_ref, y_ref, theta_ref
        data = np.append(data, [[t, test_robot.base.x, test_robot.base.y, test_robot.base.theta, ref.x, ref.y, ref.theta]], axis=0)
        t += dt

    # miscellaneous
    # col_visuals.update(eng.collision_handler.collisions)


if __name__ == '__main__':
    pyglet.clock.schedule_interval(update, 1/120)
    pyglet.app.run()

    np.savetxt("data/test.csv", data, delimiter=',')
