import numpy as np
from resources import example_robots
from misc.plot import Plot

leg_id = 1
leg_link_length = 0.8
hip_height = np.sqrt(2) * leg_link_length - 0.3

robot = example_robots.triped()
controller = robot.controller.leg_controllers[leg_id]
fcontroller = controller.flight_controller

limit = np.sqrt((2*leg_link_length)**2 - hip_height**2) - 1e-3 # limits approximated with stretched legs touching the floor at initial height
x = np.linspace(-limit, limit, num=1000)
offset = 0
if leg_id == 0: offset = -1.05
if leg_id == 2: offset = 1.05
x += offset # make the center in the position of the hip joint

q1s = []
q2s = []
svd_values = np.empty((2,0))
efforts = np.empty((2,0))
foot_forces = np.empty((2,0))
for fp in x:
    # calculate inverse kinematics
    q2 = np.arccos(((fp - offset)**2 + hip_height**2 - 2*(leg_link_length**2)) / (2 * leg_link_length**2))
    q1 = np.arctan2(fp - offset, hip_height) - np.arctan2(np.sin(q2), 1 + np.cos(q2))

    # use joint positions to simulate robot pose
    joints_state = []
    for i in range(3):
        if i == leg_id:
            joints_state.append(q1)
            joints_state.append(q2)
        else:
            joints_state.append(robot.joints[2*i].get_angle())
            joints_state.append(robot.joints[2*i+1].get_angle())
    robot.set_state(robot.base.x, hip_height + 0.2, robot.base.theta, joints_state)

    q1s.append(np.rad2deg(q1))
    q2s.append(np.rad2deg(q2))

    # calculate whatever from the robot pose using the controller
    for lc in robot.controller.leg_controllers:
        lc.state = "support"
    robot.controller.update(dt=0.01, ref=robot.base.pose)

    # jacobian = np.vstack((fcontroller.chain_frames_jacobians[-1], fcontroller.rotation_jacobians[-1]))
    jacobian = fcontroller.chain_frames_jacobians[-1]
    svd_v = np.linalg.svd(jacobian, compute_uv=False)
    svd_values = np.hstack((svd_values, svd_v[:,None]))

    efforts = np.hstack((efforts, fcontroller.efforts[:,None]))
    
    foot_force, _, _, _ = np.linalg.lstsq(fcontroller.chain_frames_jacobians[-1].T,
                                          fcontroller.efforts)
    foot_forces = np.hstack((foot_forces, foot_force[:,None]))

plot = Plot([
    (x, q1s, "q1"),
    (x, q2s, "q2"),
])

plot2 = Plot([
    (x, svd_v, fr"$SVD_{{{i}}}$") for i, svd_v in enumerate(svd_values)
])

plot3 = Plot([
    (x, tau, fr"$\tau_{{{i}}}$") for i, tau in enumerate(efforts)
])

plot4 = Plot([
    (x, foot_forces[0], fr"$f_x$"),
    (x, foot_forces[1], fr"$f_y$"),
])

input()
