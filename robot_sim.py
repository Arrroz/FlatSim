import numpy as np
from scene.scene import Scene
from scene.record import Record
from resources import example_bodies, example_robots
# from system import joint
from control.path import Path

np.set_printoptions(precision=2, suppress=True)

scene = Scene()

ground = example_bodies.ground()
scene.add_body(ground)

robot, arm_controller = example_robots.triped()
robot.base.movable = False
# ground_joint = joint.RollingContactJoint(radius=0.2, normal=np.array([0,1]), # TODO: wheel radius and ground anchor are hard-coded
#                                          parent=ground, anchor_parent=np.array([0, 25]),
#                                          child=robot.joints[-1].child, anchor_child=np.array([0, 0]),
#                                          actuated=False)
# robot.joints.append(ground_joint)
scene.add_system(robot)

scene.add_reference()
scene.reference.pose = robot.base.pose.copy()

initial_pos = arm_controller.get_end_pos()
# path = Path(
#     lambda t: (0.4 * np.array([np.sin(5*2*np.pi*t),
#                                1 - np.cos(5*2*np.pi*t)])
#                + initial_pos),
#     dist_tolerance = 0.2
# )
path = Path(
    lambda t: (1.0/(2*np.pi) * np.array([2*np.pi*t - np.sin(2*np.pi*t),
                                          1 - np.cos(2*np.pi*t)])
               + initial_pos),
    dist_tolerance = 0.5
)

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
    # robot.controller.update(dt, scene.reference.pose)
    target = path.get_target(arm_controller.get_end_pos())
    scene.reference.pos = target + robot.base.pos
    arm_controller.update(dt, target)

    scene.apply_gravity()
    scene.external_force.apply()

    record.note(dt)

    scene.engine.step(dt)

scene.run(update)

record.save("data/test.pkl")

