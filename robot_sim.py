import numpy as np
from scene.scene import Scene
from resources import example_bodies, example_robots
# from system import joint

np.set_printoptions(precision=2, suppress=True)

scene = Scene()

ground = example_bodies.ground()
scene.add_body(ground)

robot = example_robots.biped()
# ground_joint = joint.RollingContactJoint(radius=0.2, normal=np.array([0,1]), # TODO: wheel radius and ground anchor are hard-coded
#                                          parent=ground, anchor_parent=np.array([0, 25]),
#                                          child=robot.joints[-1].child, anchor_child=np.array([0, 0]),
#                                          actuated=False)
# robot.joints.append(ground_joint)
scene.add_system(robot)

scene.add_reference()
scene.reference.pose = robot.base.pose.copy()

data = np.empty(shape=(0,7))
t = 0


def update(dt):
    global data, t

    scene.reference.update(dt)
    robot.controller.update(dt, scene.reference.pose)

    scene.apply_gravity()
    scene.apply_external_force()

    scene.engine.step(dt)

    # x, y, theta, x_ref, y_ref, theta_ref
    data = np.append(data, [[t, robot.base.x, robot.base.y, robot.base.theta, scene.reference.x, scene.reference.y, scene.reference.theta]], axis=0)
    t += dt


scene.run(update)

np.savetxt("data/test.csv", data, delimiter=',')

