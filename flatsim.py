import pyglet
import numpy as np
from physics import engine, constraint, collision, solver, utils
from graphics import camera
from resources import example_robots, example_links
from misc import collision_visuals, external_force, reference

shapes_debug = False # debugging with random shapes; TODO: remove once done

# numpy setup
np.set_printoptions(precision=2, suppress=True)

# window
cam = camera.Camera(width=800, height=600)

key_handler = pyglet.window.key.KeyStateHandler()
cam.push_handlers(key_handler)

# setup bodies and constraints (constraints are in the test_robot)
ground = example_links.ground(camera=cam)

if not shapes_debug:
    test_robot, robot_controller = example_robots.wheeled_monoped(camera=cam)

if len(test_robot.joints) > 3:
    robot_type = "biped"
else:
    robot_type = "monoped"

if shapes_debug:
    circle1 = example_links.wheel(camera=cam)
    circle1.x = 2
    circle2 = example_links.wheel(camera=cam, radius=0.5)
    circle2.x = -2
    rect1 = example_links.rectangle(camera=cam)
    rect2 = example_links.rectangle(camera=cam)
    rect2.y = 1

# engine stuff
if shapes_debug:
    body_collection = collision.CollidableCollection([rect1, ground])
    #body_collection = collision.CollidableCollection([circle1, rect1, circle2, rect2, ground])
    constraint_collection = constraint.ConstraintCollection([], body_collection)
    correction_constraint_collection = constraint.ConstraintCollection([], body_collection)
if not shapes_debug:
    body_collection = collision.CollidableCollection(test_robot.links() + [ground])
    body_collection.add_collision_ignore_set(test_robot.links())
    constraint_collection = constraint.ConstraintCollection([constraint.RJointConstraint(j) for j in test_robot.joints], body_collection) # TODO: cleaner way of doing this?
    correction_constraint_collection = constraint.ConstraintCollection([constraint.RJointConstraint(j) for j in test_robot.joints], body_collection) # TODO: cleaner way of doing this?

eng = engine.Engine(constraint_collection, correction_constraint_collection, solver.LemkeSolver(), solver.LemkeSolver())

# miscellaneous
ext_force = external_force.ExternalForce(body_collection, cam)
col_visuals = collision_visuals.CollisionVisuals(camera=cam)
ref = reference.Reference(camera=cam, key_handler=key_handler)

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

    if controller_active:
        robot_controller.update(np.array([ref.x, ref.y]), np.array([ref.theta]))

    [b.apply_force(utils.gravity[0] * b.mass, utils.gravity[1] * b.mass) for b in body_collection.movables] # gravity
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
    # col_visuals.update(body_collection.collisions)



if __name__ == '__main__':
    pyglet.clock.schedule_interval(update, 1/120)
    pyglet.app.run()

    np.savetxt("data/test.csv", data, delimiter=',')
