import numpy as np
from physics import utils
from system import joint
from control.transform import Transform
from control.arm_controllers import ArmController
from control.path import Path

class SupportingLegController():

    def __init__(self, joints: list[joint.Joint], foot_radius, foot_anchor):
        self.joints = joints
        self.links = [j.child for j in joints]
        self.links.append(joints[-1].parent)

        self.foot_radius = foot_radius

        self.n_joints = len(joints)
        self.n_links = len(self.links)

        self.link_transforms = [Transform(foot_anchor)] # type: list[Transform]
        for j in self.joints:
            self.link_transforms.append(
                Transform(j.anchor_parent)
            )

        self.chain_transforms = [] # type: list[Transform]
        for i, j in enumerate(self.joints):
            self.chain_transforms.append(
                self.link_transforms[i] * Transform(-j.anchor_child, j.offset)
            )

        self.joint_velocities = np.zeros((self.n_joints+1,))

        self.chain_frames_jacobians = [np.zeros((2, self.n_links)) for _ in range(self.n_joints)]
        self.chain_frames_jacobian_derivatives = [np.zeros((2, self.n_links)) for _ in range(self.n_joints)]

        self.link_jacobians = [np.zeros((2, self.n_links)) for _ in range(self.n_links)]
        self.link_jacobian_derivatives = [np.zeros((2, self.n_links)) for _ in range(self.n_links)]

        self.rotation_jacobians = [np.zeros((1, self.n_links)) for _ in range(self.n_links)]
        self.rotation_jacobian_derivatives = [np.zeros((1, self.n_links)) for _ in range(self.n_links)]

        self.inertia = np.zeros((self.n_links, self.n_links))
        self.cc_matrix = np.zeros((self.n_links, self.n_links))

        self.gravity_efforts = np.zeros((self.n_links,))

        self.efforts = np.zeros((self.n_joints,))

    def update_matrices(self):
        # Joints' velocities
        self.joint_velocities[0] = self.links[0].w
        for i, j in enumerate(self.joints):
            self.joint_velocities[i+1] = j.parent.w - j.child.w

        # Frames
        chain_frames = [Transform()]
        link_frames = []
        curr_frame = Transform(translation=np.array([0, -self.foot_radius]),
                               angle=-self.links[0].theta) # WARNING: requires knowing the orientation of the foot link
        for i in range(self.n_joints):
            link_frames.append(curr_frame * self.link_transforms[i])
            curr_frame *= self.chain_transforms[i]
            chain_frames.append(curr_frame)
            curr_frame *= Transform(angle=self.joints[i].get_angle())
        link_frames.append(curr_frame * self.link_transforms[-1])

        # Jacobians
        for i, cf in enumerate(chain_frames[1:]):
            for j, jf in enumerate(chain_frames[:i+1]):
                derivative = cf.derivative("angle", jf)
                self.chain_frames_jacobians[i][:,j] = derivative[:2]

        for i, lf in enumerate(link_frames):
            for j, jf in enumerate(chain_frames[:i+1]):
                derivative = lf.derivative("angle", jf)
                self.link_jacobians[i][:,j] = derivative[:2]
                self.rotation_jacobians[i][0,j] = derivative[2]

        # Frames' velocities
        chain_frames_velocities = [np.array([-self.links[0].w * self.foot_radius, 0])]
        for i in range(self.n_joints):
            chain_frames_velocities.append(self.chain_frames_jacobians[i] @ self.joint_velocities)

        # Jacobians' derivatives
        for i in range(self.n_joints):
            for j in range(i+1):
                self.chain_frames_jacobian_derivatives[i][:,j] = (
                    np.array([[0, -1], [1, 0]])
                    @ (chain_frames_velocities[i+1]
                       - chain_frames_velocities[j])
                )

        for i in range(self.n_links):
            for j in range(i+1):
                self.link_jacobian_derivatives[i][:,j] = (
                    np.array([[0, -1], [1, 0]])
                    @ ((self.link_jacobians[i] @ self.joint_velocities)
                       - chain_frames_velocities[j])
                )
                self.rotation_jacobian_derivatives[i][0,j] = 0.0

        # Inertia
        self.inertia = np.zeros((self.n_links, self.n_links))
        for i in range(self.n_links):
            self.inertia += (self.link_jacobians[i].T @ self.link_jacobians[i] * self.links[i].mass
                             + self.rotation_jacobians[i].T @ self.rotation_jacobians[i] * self.links[i].moi)

        # Centrifugal and Coriolis Effects
        self.cc_matrix = np.zeros((self.n_links, self.n_links))
        for i in range(self.n_links):
            self.cc_matrix += (self.link_jacobians[i].T @ self.link_jacobian_derivatives[i] * self.links[i].mass
                               + self.rotation_jacobians[i].T @ self.rotation_jacobian_derivatives[i] * self.links[i].moi)

        # Gravity Efforts
        self.gravity_efforts = np.zeros((self.n_links,))
        for i in range(self.n_links-1):
            self.gravity_efforts += self.link_jacobians[i].T @ utils.gravity * self.links[i].mass

    def apply_efforts(self, body_wrench_ref): # check notes for explanations of these formulas
        # TODO: remove; this ignores the torque ref and allows the base to spin while keeping the center of mass in place
        # body_torque_ref = np.array([self.gravity_efforts[0] - np.dot(self.link_jacobians[-1][:,0], body_force_ref)])

        # TODO: remove; this projects the provided references in the allowed space
        # body_wrench_ref = np.block([body_force_ref, body_torque_ref]).reshape((3,1))
        # J_constraints = -np.block([[self.link_jacobians[-1][:,0:1]], [self.rotation_jacobians[-1][:,0:1]]])
        # J_constraints_T = np.transpose(J_constraints)
        # k_constraints = np.array([-self.gravity_efforts[0]]).reshape((1,1))
        # body_wrench_ref += np.matmul(np.matmul(
        #     J_constraints,
        #     np.linalg.inv(np.matmul(J_constraints_T, J_constraints))),
        #     k_constraints - np.matmul(J_constraints_T, body_wrench_ref))
        # body_force_ref = body_wrench_ref[:2,0]
        # body_torque_ref = body_wrench_ref[2:,0]

        body_efforts = (- np.vstack((self.link_jacobians[-1], self.rotation_jacobians[-1])).T
                        @ body_wrench_ref)

        self.efforts = self.gravity_efforts[1:] + body_efforts[1:] # in theory efforts = - gravity_efforts - body_efforts, but efforts holds the negatives of the torques to apply; this way, it holds the actual torques to apply
        for i in range(self.n_joints):
            self.joints[i].apply_effort(self.efforts[i])

    def update(self, body_wrench_ref):
        self.update_matrices()
        self.apply_efforts(body_wrench_ref)


class LegController():

    def __init__(self, joints: list[joint.Joint], foot_radius, foot_anchor, flight_speed=2):
        self.support_controller = SupportingLegController(joints[::-1], foot_radius=foot_radius, foot_anchor=foot_anchor)
        self.flight_controller = ArmController(joints, end_anchor=foot_anchor)

        self.flight_path = None
        self.flight_speed = flight_speed

        self.state = "support"

    def trying2pull(self):
        foot_force, _, _, _ = np.linalg.lstsq(self.flight_controller.chain_frames_jacobians[-1].T,
                                              self.flight_controller.efforts)

        return foot_force[1] > 0

    def create_flight_path(self, destination):
        initial_pos = self.flight_controller.get_end_pos()
        dist_x, dist_y = destination-initial_pos
        pi2 = 2*np.pi

        def path(t):
            cycloid_x = dist_x*t - dist_x/pi2 * np.sin(pi2*t)
            cycloid_y = dist_y*t + np.abs(dist_x)/pi2 * (1 - np.cos(pi2*t))

            return initial_pos + np.array([cycloid_x, cycloid_y])

        self.flight_path = Path(path, dist_tolerance=0.05)
        # TODO: update destination during flight?

    def setup_support(self):
        self.state = "support"
        self.flight_path = None

    def setup_flight(self):
        self.state = "flight"
        self.flight_controller.reset_contact_detection()

        # w = np.sqrt(utils.gravity[1] / self.flight_controller.get_end_pos()[1])
        # capture_point = self.flight_controller.joints[0].parent.lin_vel / w # relative to the base position

        # destination = capture_point
        # for lc in self.leg_controllers:
        #     if lc.state == "support":
        #         lc.flight_controller.update_matrices()
        #         destination -= 0.7 * (lc.flight_controller.get_end_pos() - capture_point) # TODO: check this constant
        #         print(lc.flight_controller.get_end_pos() - capture_point, end="  ")
        # destination[1] = leg_controller.flight_controller.get_end_pos()[1]
        destination = np.zeros((2,))
        destination[0] = -self.flight_controller.chain_transforms[0].translation[0]
        destination[1] = self.flight_controller.get_end_pos()[1] + 0.2
        self.create_flight_path(destination)

    def get_flight_target(self):
        if self.flight_path == None:
            raise ValueError("No flight path available")

        if self.flight_path.end_reached():
            target = self.flight_controller.get_end_pos()
            velocity = self.flight_speed * np.array([0, -1])
        else:
            target = self.flight_path.get_target(self.flight_controller.get_end_pos())
            velocity = self.flight_speed * utils.normalize(target - self.flight_controller.get_end_pos())

        return target, velocity

    def update(self, dt, ref=np.zeros((3,))):
        self.support_controller.update_matrices()
        self.flight_controller.update_matrices()

        match self.state:
            case "support":
                self.support_controller.apply_efforts(ref)
                self.flight_controller.efforts = self.support_controller.efforts[::-1]

                if self.trying2pull():
                    self.setup_flight()

            case "flight":
                target, velocity = self.get_flight_target()
                self.flight_controller.apply_efforts(dt, target, velocity)
                self.support_controller.efforts = self.flight_controller.efforts[::-1]
                self.flight_controller.update_contact_detection(dt)

                if self.flight_controller.in_contact():
                    self.setup_support()

            case _:
                raise ValueError(f"Unexpected state found in LegController: '{self.state}'; allowed states are 'support' or 'flight'")
