import numpy as np
from physics import utils
from system import joint
from control.transform import Transform
from control.pid import PIDController

class ArmController():

    def __init__(self, joints: list[joint.Joint], end_anchor):
        self.joints = joints
        self.links = [j.child for j in joints]

        self.pid = PIDController(100, 0, 20)

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

        self.dq = np.zeros((self.n_joints,))

        self.Jp = [np.zeros((2, self.n_joints)) for _ in range(self.n_links)]
        self.dJp = [np.zeros((2, self.n_joints)) for _ in range(self.n_links)]

        self.Jt = [np.zeros((2, self.n_joints)) for _ in range(self.n_links)]
        self.dJt = [np.zeros((2, self.n_joints)) for _ in range(self.n_links)]

        self.Jr = [np.zeros((1, self.n_joints)) for _ in range(self.n_links)]
        self.dJr = [np.zeros((1, self.n_joints)) for _ in range(self.n_links)]

        self.M = np.zeros((self.n_links, self.n_links))
        self.C = np.zeros((self.n_links, self.n_links))
        
        self.Fg = np.zeros((self.n_links,))
    
    def update_matrices(self):
        for i, j in enumerate(self.joints):
            self.dq[i] = j.child.w - j.parent.w

        # Jacobians
        curr_frame = self.chain_transforms[0]
        for i in range(self.n_joints):
            curr_frame *= Transform(angle=-self.joints[i].get_angle())
            curr_link_frame = curr_frame * self.link_transforms[i]
            curr_frame *= self.chain_transforms[i+1]

            joint_frame = self.chain_transforms[0]
            for j in range(i+1):
                joint_frame *= Transform(angle=-self.joints[j].get_angle())
                
                chain_derivative = curr_frame.derivative("angle", joint_frame)
                link_derivative = curr_link_frame.derivative("angle", joint_frame)

                self.Jp[i][:,j] = chain_derivative[:2]
                self.Jt[i][:,j] = link_derivative[:2]
                self.Jr[i][0,j] = link_derivative[2]
                
                joint_frame *= self.chain_transforms[j+1]

        # Jacobians' derivatives 
        chain_frames_velocities = [np.zeros((2,))]
        for i in range(self.n_joints):
            chain_frames_velocities.append(self.Jp[i] @ self.dq)
            link_frame_velocity = self.Jt[i] @ self.dq
            for j in range(i+1):
                self.dJp[i][:,j] = (np.array([[0, -1], [1, 0]])
                                    @ (chain_frames_velocities[i+1] - chain_frames_velocities[j]))
                self.dJt[i][:,j] = (np.array([[0, -1], [1, 0]])
                                    @ (link_frame_velocity - chain_frames_velocities[j]))
                self.dJr[i][0,j] = 0.0
        
        # M
        self.M = np.zeros((self.n_links, self.n_links))
        for i in range(self.n_links):
            self.M += (self.Jt[i].T @ self.Jt[i] * self.links[i].mass +
                       self.Jr[i].T @ self.Jr[i] * self.links[i].moi)
        
        # C
        self.C = np.zeros((self.n_links, self.n_links))
        for i in range(self.n_links):
            self.C += (self.Jt[i].T @ self.dJt[i] * self.links[i].mass +
                       self.Jr[i].T @ self.dJr[i] * self.links[i].moi)
        
        # Fg
        self.Fg = np.zeros((self.n_links,))
        for i in range(self.n_links):
            self.Fg += self.Jt[i].T @ utils.gravity * self.links[i].mass

    def get_end_pos(self):
        transf = self.chain_transforms[0]
        for i, ct in enumerate(self.chain_transforms[1:]):
            transf *= Transform(angle=-self.joints[i].get_angle()) * ct

        return -transf.translation

    def get_end_vel(self):
        return self.Jp[-1] @ self.dq

    def update(self, dt, ref, dref=np.zeros((2,))):
        self.update_matrices()

        acc_ref = self.pid.update(dt,
                                  ref-self.get_end_pos(),
                                  dref-self.get_end_vel())

        A_mat = self.Jp[-1] @ np.linalg.inv(self.M)
        B_mat = (self.dJp[-1] - A_mat @ self.C) @ self.dq + A_mat @ self.Fg
        
        efforts, _, _, _ = np.linalg.lstsq(A_mat, acc_ref - B_mat)
        
        for i in range(self.n_joints):
            self.joints[i].apply_effort(efforts[i])
