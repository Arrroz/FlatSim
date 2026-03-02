import numpy as np
from physics import utils, body
from control.leg_controllers import SupportingLegController
from control.pid import PIDController

class BodyController():

    def __init__(self, body: body.Body, leg_controllers: list[SupportingLegController], kp=10, ki=0.1, kd=10):
        self.body = body
        self.leg_controllers = leg_controllers
        
        z = 2
        kp = 3*z*z
        ki = z*z*z
        kd = 3*z
        # k = 3
        # z = 1
        # kp = k*2*z
        # ki = k*z*z
        # kd = k
        self.pid = PIDController(kp=kp, ki=ki, kd=kd)
        
    def update(self, dt, ref, dref=np.zeros((3,))):
        # Find pose and velocity errors
        error = ref - self.body.pose
        derror = dref - self.body.vel
        
        # Get reference acceleration as the output of a PID controller
        acc_ref = self.pid.update(dt, error, derror)

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

    def __init__(self, body: body.Body, leg_controllers: list[SupportingLegController], kp=50, ki=10, kd=20):
        self.body = body
        self.leg_controllers = leg_controllers

        self.pid = PIDController(kp=kp, ki=ki, kd=kd)
        
    def update(self, dt, ref, dref=np.zeros((3,))):
        # Find pose and velocity errors
        error = ref - self.body.pose
        derror = dref - self.body.vel
        
        # Get reference acceleration as the output of a PID controller
        acc_ref = self.pid.update(dt, error, derror)

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
                j.apply_effort(-torques[i])
                i += 1
