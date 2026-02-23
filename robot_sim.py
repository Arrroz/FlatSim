import numpy as np
from scene.scene import Scene
from scene.record import Record
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

record = Record(scene)
record.track("jacobian_left", None, lambda: np.vstack((
    robot.controller.leg_controllers[0].chain_frames_jacobians[-1],
    robot.controller.leg_controllers[0].rotation_jacobians[-1]
)))
record.track("jacobian_right", None, lambda: np.vstack((
    robot.controller.leg_controllers[1].chain_frames_jacobians[-1],
    robot.controller.leg_controllers[1].rotation_jacobians[-1]
)))

def update(dt):
    scene.reference.update(dt)
    robot.controller.update(dt, scene.reference.pose)

    scene.apply_gravity()
    scene.external_force.apply()

    record.note(dt)

    scene.engine.step(dt)

scene.run(update)

record.save("data/test.pkl")

