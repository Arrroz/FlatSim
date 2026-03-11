import numpy as np
from physics import utils
from system import joint
from control.transform import Transform
from control.pid import PIDController

class ArmController():

    def __init__(self, joints: list[joint.Joint], end_anchor, kp=100, ki=0, kd=20):
        self.joints = joints
        self.links = [j.child for j in joints]

        self.pid = PIDController(kp, ki, kd)

        self.n_joints = len(joints)
        self.n_links = len(self.links)
        
        self.link_transforms = [] # type: list[Transform]
        for j in self.joints:
            self.link_transforms.append(
                Transform(j.anchor_child)
            )

        self.chain_transforms = [] # type: list[Transform]
        for j in self.joints:
            self.chain_transforms.append(
                Transform(-j.anchor_parent, -j.offset)
            )
        self.chain_transforms.append(Transform(-end_anchor))
        for i in range(self.n_joints):
            self.chain_transforms[i+1] *= self.link_transforms[i]

        self.joint_velocities = np.zeros((self.n_joints,))

        self.chain_frames_jacobians = [np.zeros((2, self.n_joints)) for _ in range(self.n_links)]
        self.chain_frames_jacobian_derivatives = [np.zeros((2, self.n_joints)) for _ in range(self.n_links)]

        self.link_jacobians = [np.zeros((2, self.n_joints)) for _ in range(self.n_links)]
        self.link_jacobian_derivatives = [np.zeros((2, self.n_joints)) for _ in range(self.n_links)]

        self.rotation_jacobians = [np.zeros((1, self.n_joints)) for _ in range(self.n_links)]
        self.rotation_jacobian_derivatives = [np.zeros((1, self.n_joints)) for _ in range(self.n_links)]

        self.inertia = np.zeros((self.n_links, self.n_links))
        self.cc_matrix = np.zeros((self.n_links, self.n_links))
        
        self.gravity_efforts = np.zeros((self.n_links,))
    
    def update_matrices(self):
        # Joints' velocities
        for i, j in enumerate(self.joints):
            self.joint_velocities[i] = j.child.w - j.parent.w

        # Frames
        curr_frame = self.chain_transforms[0]
        chain_frames = [curr_frame]
        link_frames = []
        for i in range(self.n_joints):
            curr_frame *= Transform(angle=-self.joints[i].get_angle())
            link_frames.append(curr_frame * self.link_transforms[i])
            curr_frame *= self.chain_transforms[i+1]
            chain_frames.append(curr_frame)

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
        chain_frames_velocities = [np.zeros((2,))]
        for i in range(self.n_joints):
            chain_frames_velocities.append(self.chain_frames_jacobians[i] @ self.joint_velocities)

        # Jacobians' derivatives
        for i in range(self.n_links):
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
        for i in range(self.n_links):
            self.gravity_efforts += self.link_jacobians[i].T @ utils.gravity * self.links[i].mass

    def get_end_pos(self):
        transf = self.chain_transforms[0]
        for i, ct in enumerate(self.chain_transforms[1:]):
            transf *= Transform(angle=-self.joints[i].get_angle()) * ct

        return -transf.translation

    def get_end_vel(self):
        return self.chain_frames_jacobians[-1] @ self.joint_velocities

    def update(self, dt, ref, dref=np.zeros((2,))):
        self.update_matrices()

        acc_ref = self.pid.update(dt,
                                  ref-self.get_end_pos(),
                                  dref-self.get_end_vel())

        model_input_dynamics = self.chain_frames_jacobians[-1] @ np.linalg.inv(self.inertia)
        model_dynamics = ((self.chain_frames_jacobian_derivatives[-1] - model_input_dynamics @ self.cc_matrix) @ self.joint_velocities
                          + model_input_dynamics @ self.gravity_efforts)
        
        efforts, _, _, _ = np.linalg.lstsq(model_input_dynamics,
                                           acc_ref - model_dynamics)
        
        for i in range(self.n_joints):
            self.joints[i].apply_effort(efforts[i])
