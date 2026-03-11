import numpy as np
from physics import utils
from system import joint
from control.transform import Transform

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

        self.chain_transforms = []
        for i, j in enumerate(self.joints):
            self.chain_transforms.append(
                self.link_transforms[i] * Transform(-j.anchor_child, j.offset)
            )
        
        self.dq = np.zeros((self.n_joints+1,))

        self.Jp = [np.zeros((2, self.n_links)) for _ in range(self.n_joints)]
        self.dJp = [np.zeros((2, self.n_links)) for _ in range(self.n_joints)]

        self.Jt = [np.zeros((2, self.n_links)) for _ in range(self.n_links)]
        self.dJt = [np.zeros((2, self.n_links)) for _ in range(self.n_links)]

        self.Jr = [np.zeros((1, self.n_links)) for _ in range(self.n_links)]
        self.dJr = [np.zeros((1, self.n_links)) for _ in range(self.n_links)]

        self.M = np.zeros((self.n_links, self.n_links))
        self.C = np.zeros((self.n_links, self.n_links))
        
        self.Fg = np.zeros((self.n_links,))
    
    def update_matrices(self):
        # Joints' velocities
        self.dq[0] = self.links[0].w
        for i, j in enumerate(self.joints):
            self.dq[i+1] = j.parent.w - j.child.w

        # Frames
        chain_frames = [Transform()]
        link_frames = []
        curr_frame = Transform(translation=np.array([0, -self.foot_radius]),
                               angle=-self.links[0].theta)
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
                self.Jp[i][:,j] = derivative[:2]

        for i, lf in enumerate(link_frames):
            for j, jf in enumerate(chain_frames[:i+1]):
                derivative = lf.derivative("angle", jf)
                self.Jt[i][:,j] = derivative[:2]
                self.Jr[i][0,j] = derivative[2]

        # Frames' velocities
        chain_frames_velocities = [np.array([-self.links[0].w * self.foot_radius, 0])]
        for i in range(self.n_joints):
            chain_frames_velocities.append(self.Jp[i] @ self.dq)

        # Jacobians' derivatives 
        for i in range(self.n_joints):
            for j in range(i+1):
                self.dJp[i][:,j] = (np.array([[0, -1], [1, 0]])
                                    @ (chain_frames_velocities[i+1]
                                       - chain_frames_velocities[j]))

        for i in range(self.n_links):
            for j in range(i+1):
                self.dJt[i][:,j] = (np.array([[0, -1], [1, 0]])
                                    @ ((self.Jt[i] @ self.dq)
                                       - chain_frames_velocities[j]))
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
