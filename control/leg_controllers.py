import numpy as np
from physics import utils
from system import joint

class SupportingLegController():

    def __init__(self, joints: list[joint.Joint], foot_radius, foot_anchor):
        self.joints = joints
        self.links = [j.child for j in joints]
        self.links.append(joints[-1].parent)

        self.foot_radius = foot_radius
        
        self.n_joints = len(joints)
        self.n_links = len(self.links)
        
        self.joint_positions = [joints[0].anchor_child - foot_anchor]
        self.link_positions = [-foot_anchor]
        for i in range(self.n_joints-1):
            self.joint_positions.append(joints[i+1].anchor_child - joints[i].anchor_parent)
            self.link_positions.append(-joints[i].anchor_parent)
        self.link_positions.append(-joints[-1].anchor_parent)
        
        cross_mat = np.array([[0, -1], [1, 0]])
        self.crossed_joint_positions = [cross_mat @ p for p in self.joint_positions]
        self.crossed_link_positions = [cross_mat @ p for p in self.link_positions]

        # Jr_i = [1, 1, ..., 0, 0] (number of 1s is given by the link being referred to)
        self.Jr = [np.zeros((1, self.n_links)) for i in range(self.n_links)]
        for i in range(self.n_links):
            for j in range(i+1):
                self.Jr[i][0, j] = 1
        self.dJr = [np.zeros((1, self.n_links)) for i in range(self.n_links)]

        self.Jp = [np.zeros((2, self.n_links)) for i in range(self.n_links)]
        self.Jp[0][0,0] = self.foot_radius
        self.dJp = [np.zeros((2, self.n_links)) for i in range(self.n_links)]

        self.Jt = [np.zeros((2, self.n_links)) for i in range(self.n_links)]
        self.dJt = [np.zeros((2, self.n_links)) for i in range(self.n_links)]

        self.M = np.zeros((self.n_links, self.n_links))
        self.C = np.zeros((self.n_links, self.n_links))
        
        self.Fg = np.zeros((self.n_links,))
    
    def update_matrices(self):
        # R_i
        rot_matrices = [utils.rotation_m(l.theta) for l in self.links]

        # Jp
        for i in range(1, self.n_links):
            new_term = rot_matrices[i-1] @ self.crossed_joint_positions[i-1]
            
            # new_matrix = [new_term, new_term, ..., 0, 0] (number of 'new_term' columns is given by the joint being referred to)
            new_matrix = np.zeros((2, self.n_links))
            for j in range(i):
                new_matrix[:,j] = new_term

            self.Jp[i] = self.Jp[i-1] + new_matrix
        
        # dJp
        for i in range(1, self.n_links):
            new_term = -rot_matrices[i-1] @ self.joint_positions[i-1] * self.links[i-1].w

            # new_matrix = [new_term, new_term, ..., 0, 0] (number of 'new_term' columns is given by the joint being referred to)
            new_matrix = np.zeros((2, self.n_links))
            for j in range(i):
                new_matrix[:,j] = new_term
            
            self.dJp[i] = self.dJp[i-1] + new_matrix

        # Jt
        for i in range(len(self.Jp)):
            new_term = rot_matrices[i] @ self.crossed_link_positions[i]
            
            # new_matrix = [new_term, new_term, ..., 0, 0] (number of 'new_term' columns is given by the joint being referred to)
            new_matrix = np.zeros((2, self.n_links))
            for j in range(i+1):
                new_matrix[:,j] = new_term

            self.Jt[i] = self.Jp[i] + new_matrix
        
        # dJt
        for i in range(len(self.Jp)):
            new_term = -rot_matrices[i] @ self.link_positions[i] * self.links[i].w
            
            # new_matrix = [new_term, new_term, ..., 0, 0] (number of 'new_term' columns is given by the joint being referred to)
            new_matrix = np.zeros((2, self.n_links))
            for j in range(i+1):
                new_matrix[:,j] = new_term

            self.dJt[i] = self.dJp[i] + new_matrix
        
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
        for i in range(self.n_links-1):
            self.Fg += self.Jt[i].T @ utils.gravity * self.links[i].mass

    def update(self, body_wrench_ref): # check notes for explanations of these formulas
        self.update_matrices()
        
        # TODO: remove; this ignores the torque ref and allows the base to spin while keeping the center of mass in place
        # body_torque_ref = np.array([self.Fg[0] - np.dot(self.Jt[-1][:,0], body_force_ref)])
        
        # TODO: remove; this projects the provided references in the allowed space
        # body_wrench_ref = np.block([body_force_ref, body_torque_ref]).reshape((3,1))
        # J_constraints = -np.block([[self.Jt[-1][:,0:1]], [self.Jr[-1][:,0:1]]])
        # J_constraints_T = np.transpose(J_constraints)
        # k_constraints = np.array([-self.Fg[0]]).reshape((1,1))
        # body_wrench_ref += np.matmul(np.matmul(
        #     J_constraints,
        #     np.linalg.inv(np.matmul(J_constraints_T, J_constraints))),
        #     k_constraints - np.matmul(J_constraints_T, body_wrench_ref))
        # body_force_ref = body_wrench_ref[:2,0]
        # body_torque_ref = body_wrench_ref[2:,0]

        # Fb = - (self.Jt[-1].T @ body_force_ref + self.Jr[-1].T @ body_torque_ref)
        Fb = - np.vstack((self.Jt[-1], self.Jr[-1])).T @ body_wrench_ref
        
        Fu = self.Fg + Fb # in theory Fu = - Fg - Fb, but Fu holds the negatives of the torques to apply; this way, it holds the actual torques to apply
        for i in range(len(self.joints)):
            self.joints[i].apply_effort(Fu[i+1])

        # print(Fu) # TODO: only here for debugging
