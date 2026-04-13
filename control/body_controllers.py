import numpy as np
from physics import utils, body
from control.leg_controllers import LegController, SupportingLegController
from control.pid import PIDController

import pyglet

class BodyController():

    def __init__(self, body: body.Body, leg_controllers: list[LegController], kp=20, ki=0, kd=20):
        self.body = body
        self.leg_controllers = leg_controllers
        
        self.pid = PIDController(kp=kp, ki=ki, kd=kd)

        self.leg_wrenches = []

    def on_key_press(self, symbol, modifiers):
        if symbol != pyglet.window.key.M:
            return

        self.reposition_leg()

    def reposition_leg(self):
        # If any leg is in flight, don't lift any other
        # for lc in self.leg_controllers:
        #     if lc.state == "flight":
        #         return
        
        # Find leg to lift
        # foot_y_forces = []
        # for lc in self.leg_controllers:
        #     lc.flight_controller.update_matrices()

        #     # TODO: make efforts universal to the controller?
        #     efforts = lc.support_controller.efforts[::-1] if lc.state == "support" else lc.flight_controller.efforts

        #     foot_force, _, _, _ = np.linalg.lstsq(lc.flight_controller.chain_frames_jacobians[-1].T,
        #                                           efforts)
            
        #     foot_y_forces.append(foot_force[1])

        # leg_id = foot_y_forces.index(max(foot_y_forces))
        # leg_id=1

        jacobians = [np.vstack((
                        lc.support_controller.chain_frames_jacobians[-1],
                        lc.support_controller.rotation_jacobians[-1]
                     )) for lc in self.leg_controllers if lc.state == "support"]
        max_vs = [max(np.linalg.svd(j, compute_uv=False)) for j in jacobians]
        leg_id = max_vs.index(max(max_vs))
        # leg_id = 1 # TODO: remove; only here to test stuff with the middle leg

        # Setup for flight
        self.leg_controllers[leg_id].setup_flight()
        # if self.leg_controllers[leg_id].state == "support":
        #     self.leg_controllers[leg_id].setup_flight()
        # else:
        #     self.leg_controllers[leg_id].setup_support()
        
    def update(self, dt, ref, dref=np.zeros((3,))):
        # Find pose and velocity errors
        error = ref - self.body.pose
        derror = dref - self.body.vel
        
        # Get reference acceleration as the output of a PID controller
        acc_ref = self.pid.update(dt, error, derror)

        # Get required wrench as a function of the reference acceleration
        wrench_ref = (np.diag([self.body.mass, self.body.mass, self.body.moi]) @
                      (acc_ref - np.concatenate((utils.gravity, [0]))))

        # Discriminate leg controllers
        support_controllers = [lc for lc in self.leg_controllers if lc.state == "support"]
        flight_controllers = [lc for lc in self.leg_controllers if lc.state != "support"]

        # Solve the wrench distribution problem for supporting legs
        num_support_legs = len(support_controllers)

        # if num_support_legs == 1: # monoped # TODO: check if necessary
        #     support_controllers[0].update(wrench_ref)
        #     return

        JL = np.zeros((num_support_legs, 3*num_support_legs))
        kL = np.zeros((num_support_legs,))
        for i in range(num_support_legs):
            lc = support_controllers[i]
            lc.support_controller.update_matrices() # TODO: requiring the jacobians to be calculcated by the legs first is annoying for the distributed control

            Jl = np.block([lc.support_controller.link_jacobians[-1].T,
                           lc.support_controller.rotation_jacobians[-1].T])
            JL[i, 3*i:3*(i+1)] = Jl[0, :]

            kl = lc.support_controller.gravity_efforts # TODO: implement M and N components compensation
            kL[i] = kl[0]

        gamma = np.tile(np.eye(3), num_support_legs)
        
        ls_vec = np.concatenate((wrench_ref, kL))
        ls_mat = np.concatenate([gamma, JL])

        wrenches, _, _, _ = np.linalg.lstsq(ls_mat, ls_vec)

        self.leg_wrenches = []
        for i in range(num_support_legs):
            wrench = wrenches[3*i:3*(i+1)]
            self.leg_wrenches.append(wrench)

            support_controllers[i].update(dt, ref=wrench)

        # Move legs in flight
        for lc in flight_controllers:
            lc.update(dt)


class WholeBodyController(): # TODO: change to work with general LegControllers

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
        ls_mat = np.zeros((3,0))
        parasite_influences = np.zeros((3,))

        for lc in self.leg_controllers:
            lc.update_matrices() # TODO: requiring the jacobians to be calculcated by the legs first is annoying for the distributed control

            body_jacobian = np.vstack((lc.link_jacobians[-1], lc.rotation_jacobians[-1]))
            body_jacobian_derivative = np.vstack((lc.link_jacobian_derivatives[-1], lc.rotation_jacobian_derivatives[-1]))

            leg_input_dynamics = body_jacobian @ np.linalg.inv(lc.inertia)

            ls_mat = np.hstack((ls_mat, leg_input_dynamics[:,1:]))
            parasite_influences += ((body_jacobian_derivative - leg_input_dynamics @ lc.cc_matrix) @ lc.joint_velocities
                                    + leg_input_dynamics @ lc.gravity_efforts)

        lc = self.leg_controllers[0] # TODO: the gravity influence should be leg agnostic
        body_jacobian = np.vstack((lc.link_jacobians[-1], lc.rotation_jacobians[-1]))
        gravity_influence = np.concatenate((utils.gravity, [0])) # WARNING: this is actually much more complex than shown and very complicated to decompose in such a way that this is one of the terms
        # gravity_influence = body_jacobian @ np.linalg.inv(lc.inertia) @ body_jacobian.T @ np.concatenate((utils.gravity, [0])) * self.body.mass
        
        ls_vec = acc_ref - parasite_influences - gravity_influence

        torques, _, _, _ = np.linalg.lstsq(ls_mat, ls_vec)

        i = 0
        for lc in self.leg_controllers:
            for j in lc.joints:
                j.apply_effort(-torques[i])
                i += 1
