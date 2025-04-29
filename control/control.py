import numpy as np
from physics import utils
from robot import joint, link

class LegController():

    def __init__(self, joints: list[joint.Joint], foot_radius, foot_anchor):
        self.joints = joints
        self.foot_radius = foot_radius
        self.n_joints = len(joints)
        self.n_links = self.n_joints + 1
        
        self.joint_positions = [joints[0].anchor_child - foot_anchor]
        self.link_positions = [-foot_anchor]
        for i in range(self.n_joints-1):
            self.joint_positions.append(joints[i+1].anchor_child - joints[i].anchor_parent)
            self.link_positions.append(-joints[i].anchor_parent)
        self.link_positions.append(-joints[-1].anchor_parent)
        
        cross_mat = np.array([[0, -1], [1, 0]])
        self.crossed_joint_positions = [np.matmul(cross_mat, p) for p in self.joint_positions]
        self.crossed_link_positions = [np.matmul(cross_mat, p) for p in self.link_positions]

        # Jr_i = [1, 1, ..., 0, 0] (number of 1s is given by the link being referred to)
        self.Jr = [] # type: list[np.ndarray]
        for i in range(self.n_links):
            new_Jr = np.zeros((1, self.n_links))
            for j in range(i+1):
                new_Jr[0, j] = 1

            self.Jr.append(new_Jr)
        
        self.Jp = [np.zeros((2, self.n_links))] # type: list[np.ndarray]
        self.Jp[0][0,0] = self.foot_radius
        for i in range(self.n_joints):
            self.Jp.append(np.zeros((2, self.n_links)))

        self.Jt = [] # type: list[np.ndarray]
        for i in range(self.n_links):
            self.Jt.append(np.zeros((2, self.n_links)))
        
        self.Fg = np.zeros((self.n_links,))
    
    def update_matrices(self):
        # R_i
        rot_matrices = [utils.rotation_m(self.joints[0].child.theta)]
        for i in range(self.n_joints):
            rot_matrices.append(utils.rotation_m(self.joints[i].parent.theta))

        # Jp
        for i in range(1, self.n_links):
            # new_term = R_i-1 * [x] * l_i-1
            new_term = np.matmul(rot_matrices[i-1], self.crossed_joint_positions[i-1])
            
            # new_matrix = [new_term, new_term, ..., 0, 0] (number of 'new_term' columns is given by the joint being referred to)
            new_matrix = np.zeros((2, self.n_links))
            for j in range(i):
                new_matrix[:,j] = new_term

            # Jp_i = Jp_i-1 + new_matrix
            self.Jp[i] = self.Jp[i-1] + new_matrix

        # Jt
        for i in range(len(self.Jp)):
            # new_term = R_i * [x] * l_ic
            new_term = np.matmul(rot_matrices[i], self.crossed_link_positions[i])
            
            # new_matrix = [new_term, new_term, ..., 0, 0] (number of 'new_term' columns is given by the joint being referred to)
            new_matrix = np.zeros((2, len(self.joints)+1))
            for j in range(i+1):
                new_matrix[:,j] = new_term

            # Jt_i = Jp_i + new_matrix
            self.Jt[i] = self.Jp[i] + new_matrix
        
        # Fg
        self.Fg = np.zeros((self.n_links,))
        for i in range(self.n_joints):
            # Fg = sum(Jt_i^T * m_i * g)
            self.Fg += np.matmul(np.transpose(self.Jt[i]), utils.gravity) * self.joints[i].child.mass
        
        # TODO: implement M and N components compensation

    def update(self, body_force_ref, body_torque_ref): # check notes for explanations of these formulas
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

        # Fb = - Jt_-1^T * f_b - Jr_-1^T * tau_b
        Fb = - (np.matmul(np.transpose(self.Jt[-1]), body_force_ref) +
                np.matmul(np.transpose(self.Jr[-1]), body_torque_ref))
        
        Fu = self.Fg + Fb # in theory Fu = - Fg - Fb, but Fu holds the negatives of the torques to apply; this way, it holds the actual torques to apply
        for i in range(len(self.joints)):
            self.joints[i].parent.apply_torque(-Fu[i+1])
            self.joints[i].child.apply_torque(Fu[i+1])

        # print(Fu) # TODO: only here for debugging


class BodyController():
    def __init__(self, body: link.Link, leg_controllers: list[LegController], kp=50, kd=50):
        self.body = body
        self.leg_controllers = leg_controllers
        self.kp = kp
        self.kd = kd
        self.int_pos_error = np.array([0.0, 0.0]) # TODO: testing integral component
        self.int_theta_error = np.array([0.0]) # TODO: testing integral component
        
    def update(self, pos_ref, theta_ref):
        # Find pose and velocity errors
        pos_error = pos_ref - np.array([self.body.x, self.body.y])
        vel_error = -np.array([self.body.vx, self.body.vy])
        theta_error = theta_ref - np.array([self.body.theta])
        w_error = -np.array([self.body.w])

        # TODO: testing integral component
        self.int_pos_error += pos_error
        self.int_theta_error += theta_error
        ki = 0.2

        # Get required wrench as the output of a PD controller
        force_ref = self.kp * pos_error + self.kd * vel_error# + ki * self.int_pos_error
        torque_ref = self.kp * theta_error + self.kd * w_error# + ki * self.int_theta_error
        force_ref += - self.body.mass * utils.gravity

        # Solve the wrench distribution problem
        nLegs = len(self.leg_controllers)

        if nLegs == 1: # monoped
            self.leg_controllers[0].update(force_ref, torque_ref)
            return

        JL = np.zeros((nLegs, 3*nLegs))
        kL = np.zeros((nLegs,))
        for i in range(nLegs):
            lc = self.leg_controllers[i]
            lc.update_matrices() # TODO: requiring the jacobians to be calculcated by the legs first is annoying for the distributed control

            Jl = np.block([lc.Jt[-1].T, lc.Jr[-1].T])
            JL[i, 3*i:3*(i+1)] = Jl[0, :]

            kl = lc.Fg # TODO: implement M and N components compensation
            kL[i] = kl[0]

        gamma = np.tile(np.eye(3), nLegs)
        
        lsVec = np.concatenate((force_ref, torque_ref, kL))
        lsMat = np.concatenate([gamma, JL])

        wrenches, _, _, _ = np.linalg.lstsq(lsMat, lsVec)

        for i in range(nLegs):
            force = wrenches[3*i:3*i+2]
            torque = wrenches[3*i+2:3*i+3]

            self.leg_controllers[i].update(force, torque)


class TorqueController():

    def __init__(self, key_handler, torque=1):
        self.torque = torque
        self.key_handler = key_handler
        self.joint_handlers = []

    def setup_handler(self, joint: joint.Joint, positive_key, negative_key):
        self.joint_handlers.append(JointHandler(joint, positive_key, negative_key))

    def update(self):
        for h in self.joint_handlers:
            if self.key_handler[h.positive_key]:
                h.joint.parent.apply_torque(-self.torque)
                h.joint.child.apply_torque(self.torque)
            if self.key_handler[h.negative_key]:
                h.joint.parent.apply_torque(self.torque)
                h.joint.child.apply_torque(-self.torque)

class VelocityController():

    def __init__(self, key_handler, speed=np.pi/2):
        self.speed = speed
        self.key_handler = key_handler
        self.joint_handlers = []

    def setup_handler(self, joint: joint.Joint, positive_key, negative_key):
        self.joint_handlers.append(JointHandler(joint, positive_key, negative_key))

    def update(self, dt):
        for h in self.joint_handlers:
            if self.key_handler[h.positive_key]:
                h.joint.set_angle(h.joint.get_angle() + self.speed*dt)
            if self.key_handler[h.negative_key]:
                h.joint.set_angle(h.joint.get_angle() - self.speed*dt)


class JointHandler():

    def __init__(self, joint: joint.Joint, positive_key, negative_key):
        self.joint = joint
        self.positive_key = positive_key
        self.negative_key = negative_key
