import numpy as np
from scene.scene import Scene
from scene.record import Record
from resources import example_bodies, example_robots
# from system import joint

np.set_printoptions(precision=2, suppress=True)

scene = Scene()

ground = example_bodies.ground()
scene.add_body(ground)

robot = example_robots.triped()
# ground_joint = joint.RollingContactJoint(radius=0.2, normal=np.array([0,1]), # TODO: wheel radius and ground anchor are hard-coded
#                                          parent=ground, anchor_parent=np.array([0, 25]),
#                                          child=robot.joints[-1].child, anchor_child=np.array([0, 0]),
#                                          actuated=False)
# robot.joints.append(ground_joint)
scene.add_system(robot)

scene.add_reference()
scene.reference.pose = robot.base.pose.copy()

scene.camera.push_handlers(on_key_press=robot.controller.on_key_press)

lc = robot.controller.leg_controllers[1]

record = Record(scene)
record.track("torques", None,
             lambda: lc.support_controller.efforts[::-1] if lc.state == "support" else lc.flight_controller.efforts)
record.track("jacobian", None, lambda: np.vstack((
    lc.flight_controller.chain_frames_jacobians[-1],
    lc.flight_controller.rotation_jacobians[-1]
)))
record.track("phases", None, lambda: [lc.state for lc in robot.controller.leg_controllers])
record.track("velocity", None,
             lc.flight_controller.get_end_vel)
record.track("residual", None,
             lambda: lc.flight_controller._efforts_residual)
# record.track("jacobian_left", None, lambda: np.vstack((
#     robot.controller.leg_controllers[0].support_controller.chain_frames_jacobians[-1],
#     robot.controller.leg_controllers[0].support_controller.rotation_jacobians[-1]
# )))
# record.track("jacobian_middle", None, lambda: np.vstack((
#     robot.controller.leg_controllers[1].support_controller.chain_frames_jacobians[-1],
#     robot.controller.leg_controllers[1].support_controller.rotation_jacobians[-1]
# )))
# record.track("jacobian_right", None, lambda: np.vstack((
#     robot.controller.leg_controllers[2].support_controller.chain_frames_jacobians[-1],
#     robot.controller.leg_controllers[2].support_controller.rotation_jacobians[-1]
# )))

def update(dt):
    scene.reference.update(dt)
    robot.controller.update(dt, scene.reference.pose)

    scene.apply_gravity()
    scene.external_force.apply()

    record.note(dt)

    scene.engine.step(dt)

scene.run(update)

record.save("data/test.pkl")

