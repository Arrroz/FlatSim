import numpy as np
from scene.scene import Scene
from scene.record import Record
from resources import example_bodies, example_robots
# from system import joint

np.set_printoptions(precision=2, suppress=True)

scene = Scene()

ground = example_bodies.ground()
scene.add_body(ground)

# robot = example_robots.biped()
robot, arm_controller = example_robots.triped()
robot.base.movable = False
# ground_joint = joint.RollingContactJoint(radius=0.2, normal=np.array([0,1]), # TODO: wheel radius and ground anchor are hard-coded
#                                          parent=ground, anchor_parent=np.array([0, 25]),
#                                          child=robot.joints[-1].child, anchor_child=np.array([0, 0]),
#                                          actuated=False)
# robot.joints.append(ground_joint)
scene.add_system(robot)
robot_ref = robot.base.pose.copy()

scene.add_reference()
# scene.reference.pose = robot.base.pose.copy()
scene.reference.pos = robot.base.pos + arm_controller.get_end_pos()

record = Record(scene)
record.track("jacobian", None, lambda: robot.controller.leg_controllers[0].Jt[-1])

def update(dt):
    record.note(dt)

    scene.reference.update(dt)
    robot.controller.update(dt, robot_ref)
    arm_controller.update(dt, scene.reference.pos-robot.base.pos)

    scene.apply_gravity()
    scene.external_force.apply()

    scene.engine.step(dt)

scene.run(update)

record.save("data/test.pkl")

