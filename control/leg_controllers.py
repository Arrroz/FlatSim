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
    
    def update_matrices(self):
        # Joints' velocities
        self.joint_velocities[0] = self.links[0].w
        for i, j in enumerate(self.joints):
            self.joint_velocities[i+1] = j.parent.w - j.child.w

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

    def update(self, body_wrench_ref): # check notes for explanations of these formulas
        self.update_matrices()
        
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

        # body_efforts = - (self.link_jacobians[-1].T @ body_force_ref + self.rotation_jacobians[-1].T @ body_torque_ref)
        body_efforts = - np.vstack((self.link_jacobians[-1], self.rotation_jacobians[-1])).T @ body_wrench_ref
        
        joint_efforts = self.gravity_efforts + body_efforts # in theory joint_efforts = - gravity_efforts - body_efforts, but joint_efforts holds the negatives of the torques to apply; this way, it holds the actual torques to apply
        for i in range(len(self.joints)):
            self.joints[i].apply_effort(joint_efforts[i+1])

        # print(joint_efforts) # TODO: only here for debugging
