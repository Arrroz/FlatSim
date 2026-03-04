import numpy as np
from physics import utils
from system import joint
from control.pid import PIDController

class ArmController():

    def __init__(self, joints: list[joint.Joint], end_anchor):
        self.joints = joints
        self.links = [j.child for j in joints]

        self.end_anchor = end_anchor

        self.pid = PIDController(100, 0, 20)

        self.n_joints = len(joints)
        self.n_links = len(self.links)
        
        self.joint_positions = []
        self.link_positions = []
        for i in range(self.n_joints-1):
            self.joint_positions.append(joints[i+1].anchor_parent - joints[i].anchor_child)
            self.link_positions.append(-joints[i].anchor_child)
        self.joint_positions.append(end_anchor - joints[-1].anchor_child)
        self.link_positions.append(-joints[-1].anchor_child)
        
        cross_mat = np.array([[0, -1], [1, 0]])
        self.crossed_joint_positions = [cross_mat @ p for p in self.joint_positions]
        self.crossed_link_positions = [cross_mat @ p for p in self.link_positions]

        # Jr_i = [1, 1, ..., 0, 0] (number of 1s is given by the link being referred to)
        self.Jr = [np.zeros((1, self.n_joints))] * self.n_links
        for i in range(self.n_links):
            for j in range(i+1):
                self.Jr[i][0, j] = 1
        self.dJr = [np.zeros((1, self.n_joints))] * self.n_links

        self.Jp = [np.zeros((2, self.n_joints))] * self.n_links
        self.dJp = [np.zeros((2, self.n_joints))] * self.n_links

        self.Jt = [np.zeros((2, self.n_joints))] * self.n_links
        self.dJt = [np.zeros((2, self.n_joints))] * self.n_links

        self.M = np.zeros((self.n_links, self.n_links))
        self.C = np.zeros((self.n_links, self.n_links))
        
        self.Fg = np.zeros((self.n_links,))
    
    def update_matrices(self):
        # R_i
        rot_matrices = [utils.rotation_m(l.theta) for l in self.links]

        # Jp
        self.Jp[0][:,0] = rot_matrices[0] @ self.crossed_joint_positions[0]
        for i in range(1, self.n_links):
            new_term = rot_matrices[i] @ self.crossed_joint_positions[i]
            
            # new_matrix = [new_term, new_term, ..., 0, 0] (number of 'new_term' columns is given by the joint being referred to)
            new_matrix = np.zeros((2, self.n_links))
            for j in range(i+1):
                new_matrix[:,j] = new_term

            self.Jp[i] = self.Jp[i-1] + new_matrix
        
        # dJp
        self.dJp[0][:,0] = -rot_matrices[0] @ self.joint_positions[0] * self.links[0].w
        for i in range(1, self.n_links):
            new_term = -rot_matrices[i] @ self.joint_positions[i] * self.links[i].w

            # new_matrix = [new_term, new_term, ..., 0, 0] (number of 'new_term' columns is given by the joint being referred to)
            new_matrix = np.zeros((2, self.n_links))
            for j in range(i):
                new_matrix[:,j] = new_term
            
            self.dJp[i] = self.dJp[i-1] + new_matrix

        # Jt
        self.Jt[0][:,0] = rot_matrices[0] @ self.crossed_link_positions[0]
        for i in range(1, self.n_links):
            new_term = rot_matrices[i] @ self.crossed_link_positions[i]
            
            # new_matrix = [new_term, new_term, ..., 0, 0] (number of 'new_term' columns is given by the joint being referred to)
            new_matrix = np.zeros((2, self.n_links))
            for j in range(i+1):
                new_matrix[:,j] = new_term

            self.Jt[i] = self.Jp[i-1] + new_matrix
        
        # dJt
        self.dJt[0][:,0] = -rot_matrices[0] @ self.link_positions[0] * self.links[0].w
        for i in range(1, self.n_links):
            new_term = -rot_matrices[i] @ self.link_positions[i] * self.links[i].w
            
            # new_matrix = [new_term, new_term, ..., 0, 0] (number of 'new_term' columns is given by the joint being referred to)
            new_matrix = np.zeros((2, self.n_links))
            for j in range(i+1):
                new_matrix[:,j] = new_term

            self.dJt[i] = self.dJp[i-1] + new_matrix
        
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
        end_pos = np.zeros((2,))
        for j, p in zip(self.joints[::-1], self.joint_positions[::-1]):
            end_pos = (utils.rotation_m(j.get_angle() + j.offset)
                       @ (end_pos + p))
        end_pos += self.joints[0].anchor_parent

        return end_pos

    def get_end_vel(self):
        dq = [j.child.w - j.parent.w for j in self.joints]
        return self.Jp[-1] @ dq

    def update(self, dt, ref, dref=np.zeros((2,))):
        self.update_matrices()

        acc_ref = self.pid.update(dt,
                                  ref-self.get_end_pos(),
                                  dref-self.get_end_vel())

        dql = [j.child.w - j.parent.w for j in self.joints]
        A_mat = self.Jp[-1] @ np.linalg.inv(self.M)
        B_mat = (self.dJp[-1] - A_mat @ self.C) @ dql + A_mat @ self.Fg
        
        efforts, _, _, _ = np.linalg.lstsq(A_mat, acc_ref - B_mat)
        
        for i in range(len(self.joints)):
            self.joints[i].apply_effort(efforts[i])
