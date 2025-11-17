import numpy as np
from physics import utils
from robot import joint, link

class LegController():

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
            self.joints[i].parent.apply_torque(-Fu[i+1])
            self.joints[i].child.apply_torque(Fu[i+1])

        # print(Fu) # TODO: only here for debugging


class BodyController():

    def __init__(self, body: link.Link, leg_controllers: list[LegController], kp=10, kd=10):
        self.body = body
        self.leg_controllers = leg_controllers
        self.kp = kp
        self.kd = kd
        self.int_error = np.zeros((3,)) # TODO: testing integral component
        
    def update(self, dt, ref, dref=np.zeros((3,))):
        # Find pose and velocity errors
        error = ref - self.body.pose
        derror = dref - self.body.vel
        # TODO: testing integral component
        self.int_error += error * dt
        # ki = 0.1
        z = 2
        kp = 3*z*z
        ki = z*z*z
        kd = 3*z
        # k = 3
        # z = 1
        # kp = k*2*z
        # ki = k*z*z
        # kd = k

        # Get reference acceleration as the output of a PD controller
        # acc_ref = self.kp * error + self.kd * derror + ki * self.int_error
        acc_ref = kp * error + ki * self.int_error + kd * derror

        # Get required wrench as a function of the reference acceleration
        wrench_ref = (np.diag([self.body.mass, self.body.mass, self.body.moi]) @
                      (acc_ref - np.concatenate((utils.gravity, [0]))))

        # Solve the wrench distribution problem
        nLegs = len(self.leg_controllers)

        if nLegs == 1: # monoped
            self.leg_controllers[0].update(wrench_ref)
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
        
        lsVec = np.concatenate((wrench_ref, kL))
        lsMat = np.concatenate([gamma, JL])

        wrenches, _, _, _ = np.linalg.lstsq(lsMat, lsVec)

        for i in range(nLegs):
            wrench = wrenches[3*i:3*(i+1)]

            self.leg_controllers[i].update(wrench)


class WholeBodyController():

    def __init__(self, body: link.Link, leg_controllers: list[LegController], kp=5, kd=10):
        self.body = body
        self.leg_controllers = leg_controllers
        self.kp = kp
        self.kd = kd
        self.int_error = np.zeros((3,)) # TODO: testing integral component
        
    def update(self, dt, ref, dref=np.zeros((3,))):
        # Find pose and velocity errors
        error = ref - self.body.pose
        derror = dref - self.body.vel
        # TODO: testing integral component
        self.int_error += error * dt
        ki = 0.2

        # Get reference acceleration as the output of a PD controller
        acc_ref = self.kp * error + self.kd * derror# + ki * self.int_error

        # Solve the leg influence distribution problem
        lsMat = np.zeros((3,0))
        parasite_influences = np.zeros((3,))

        for lc in self.leg_controllers:
            lc.update_matrices() # TODO: requiring the jacobians to be calculcated by the legs first is annoying for the distributed control

            dql = np.zeros((lc.n_joints+1,))
            dql[0] = lc.links[0].w
            for i in range(1, lc.n_links):
                dql[i] = lc.links[i].w - lc.links[i-1].w
            
            Jb = np.vstack((lc.Jt[-1], lc.Jr[-1]))
            dJb = np.vstack((lc.dJt[-1], lc.dJr[-1]))

            Amat = Jb @ np.linalg.inv(lc.M)

            lsMat = np.hstack((lsMat, Amat[:,1:]))
            parasite_influences += (dJb - Amat @ lc.C) @ dql + Amat @ lc.Fg

        lc = self.leg_controllers[0] # TODO: the gravity influence should be leg agnostic
        Jb = np.vstack((lc.Jt[-1], lc.Jr[-1]))
        gravity_influence = np.concatenate((utils.gravity, [0])) # WARNING: this is actually much more complex than shown and very complicated to decompose in such a way that this is one of the terms
        # gravity_influence = Jb @ np.linalg.inv(lc.M) @ Jb.T @ np.concatenate((utils.gravity, [0])) * self.body.mass
        
        lsVec = acc_ref - parasite_influences - gravity_influence

        torques, _, _, _ = np.linalg.lstsq(lsMat, lsVec)

        i = 0
        for lc in self.leg_controllers:
            for j in lc.joints:
                j.parent.apply_torque(torques[i])
                j.child.apply_torque(-torques[i])
                i += 1


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
