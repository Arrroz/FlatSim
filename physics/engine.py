import numpy as np
from physics import body, solver, constraint, collision

class Engine():

    def __init__(self, bodies: list[body.Body], constraints: list[constraint.Constraint],
                 integration_solver: solver.Solver = solver.LemkeSolver(), drift_solver: solver.Solver = solver.LemkeSolver(),
                 min_sub_dt=1e-4, max_drift_iterations=3):
        self.bodies = bodies
        self.constraints = constraints
        self.integration_solver = integration_solver
        self.drift_solver = drift_solver
        self.min_sub_dt = min_sub_dt
        self.max_drift_iterations = max_drift_iterations # correct_drift is a linear approximation and can overshoot into penetration; retry (re-detecting collisions each time) until resolved or this cap is hit

        self.constraint_handler = constraint.ConstraintHandler(self.constraints, self.bodies)
        self.correction_constraint_handler = constraint.ConstraintHandler(self.constraints, self.bodies)
        self.collision_handler = collision.CollisionHandler(self.bodies)

        self.reset()

    def reset(self):
        self.movables = [b for b in self.bodies if b.movable]

        self.constraint_handler.reset(self.constraints, self.bodies)
        self.correction_constraint_handler.reset(self.constraints, self.bodies)

        self.collision_handler.reset(self.bodies)

        self.reset_solver_matrices()

    def reset_solver_matrices(self):
        mat = self.constraint_handler.matrices
        c_mat = self.correction_constraint_handler.matrices

        self.integration_solver.M = np.block([[mat.M, np.zeros((mat.M_dim, mat.C_dim))],
                                              [np.zeros((mat.C_dim, mat.M_dim + mat.C_dim))]])
        self.drift_solver.M = np.block([[c_mat.M, np.zeros((c_mat.M_dim, c_mat.C_dim))],
                                        [np.zeros((c_mat.C_dim, c_mat.M_dim + c_mat.C_dim))]])

        self.integration_solver.q = np.zeros((mat.M_dim + mat.C_dim,))
        self.drift_solver.q = np.zeros((c_mat.M_dim + c_mat.C_dim,))

        # the dynamics rows (the top M_dim rows/cols) are always equality; the rest follow each constraint's own flag
        equalities = np.array([c.equality for c in self.constraint_handler.constraints for _ in range(c.dimension)], dtype=bool)
        c_equalities = np.array([c.equality for c in self.correction_constraint_handler.constraints for _ in range(c.dimension)], dtype=bool)
        self.integration_solver.eq_mask = np.block([np.ones((mat.M_dim,), dtype=bool), equalities])
        self.drift_solver.eq_mask = np.block([np.ones((c_mat.M_dim,), dtype=bool), c_equalities])

    def integrate(self, dt):
        # update necessary constraint matrices
        self.constraint_handler.update_jacobians()
        self.constraint_handler.update_forces_vector()
        self.constraint_handler.update_constraint_constants()
        mat = self.constraint_handler.matrices

        # load each body's current velocity into dq; the right-hand side below reads it,
        # then the solver overwrites dq with the updated velocity
        for i in range(len(self.movables)):
            mat.dq[3*i:3*i+3] = self.movables[i].vel

        # update solver matrices as needed
        solver = self.integration_solver
        #if solver.q.shape[0] != mat.M_dim + mat.C_dim: # TODO: reinsert this; currently, inserting this makes M not update and have wrong dimensions because of the way the E and miu matrices are added to M
        self.reset_solver_matrices()

        # extra rows and columns of solver.M come from friction constraints (each adds a slack variable
        # coupling its own two tangent rows to its own contact's normal-force row) # TODO: this is sooooooo ugly *barf*
        # constraints can be in any order, so each coupling is placed at the constraints' own row offsets
        # rather than assumed slots, matching a friction constraint to its contact via their shared collision
        offset = mat.M_dim
        friction_offsets = {}
        contact_offsets = {}
        for c in self.constraint_handler.constraints:
            if isinstance(c, constraint.FrictionConstraint):
                friction_offsets[c] = offset
            elif isinstance(c, constraint.ContactConstraint):
                contact_offsets[c.collision] = offset
            offset += c.dimension

        n_friction_constraints = len(friction_offsets)
        extra_columns = np.zeros((mat.M_dim + mat.C_dim, n_friction_constraints))
        extra_rows = np.zeros((n_friction_constraints, mat.M_dim + mat.C_dim + n_friction_constraints))
        for i, (friction_constraint, friction_offset) in enumerate(friction_offsets.items()):
            extra_columns[friction_offset:friction_offset+friction_constraint.dimension, i] = 1
            extra_rows[i, contact_offsets[friction_constraint.collision]] = friction_constraint.friction_coefficient
            extra_rows[i, friction_offset:friction_offset+friction_constraint.dimension] = -1

        solver.M[:mat.M_dim, mat.M_dim:] = -np.transpose(mat.J)
        solver.M[mat.M_dim:, :mat.M_dim] = mat.J
        solver.q[:mat.M_dim] = -(mat.M @ mat.dq + dt * mat.F)
        solver.q[mat.M_dim:] = mat.k

        solver.M = np.block([[solver.M, extra_columns],
                             [extra_rows]])
        solver.q = np.block([solver.q, np.zeros((n_friction_constraints,))])
        solver.eq_mask = np.block([solver.eq_mask, np.zeros((n_friction_constraints,), dtype=bool)]) # friction slack rows are inequality

        # solve for the velocities
        sol = solver.solve()
        if sol != None:
            sol = sol[1]
            mat.dq = sol[:mat.M_dim]

        # perform the integration
        for i in range(len(self.movables)):
            self.movables[i].vx = mat.dq[3*i]
            self.movables[i].vy = mat.dq[3*i+1]
            self.movables[i].w = mat.dq[3*i+2]

            self.movables[i].x += dt * self.movables[i].vx
            self.movables[i].y += dt * self.movables[i].vy
            self.movables[i].theta += dt * self.movables[i].w

            self.movables[i].rfx = 0
            self.movables[i].rfy = 0
            self.movables[i].rtorque = 0

    def correct_drift(self):
        for _ in range(self.max_drift_iterations):
            # update necessary constraint matrices
            self.correction_constraint_handler.update_jacobians()
            self.correction_constraint_handler.update_constraint_errors()
            mat = self.correction_constraint_handler.matrices

            # update solver matrices as needed
            solver = self.drift_solver
            if solver.q.shape[0] != mat.M_dim + mat.C_dim:
                self.reset_solver_matrices()

            solver.M[:mat.M_dim, mat.M_dim:] = -np.transpose(mat.J)
            solver.M[mat.M_dim:, :mat.M_dim] = mat.J
            solver.q[mat.M_dim:] = mat.e

            # solve for the changes in state
            _, sol, _ = solver.solve()
            delta_q = sol[:mat.M_dim]

            # apply those changes
            for i in range(len(self.movables)):
                self.movables[i].x += delta_q[3*i]
                self.movables[i].y += delta_q[3*i+1]
                self.movables[i].theta += delta_q[3*i+2]

            # the correction is a linear approximation and can overshoot into penetration (or leave it
            # unresolved); re-detect so contacts reflect the corrected positions, then retry if still penetrating
            self.update_collision_constraints()
            if not any(c.dist < 0 for c in self.collision_handler.collisions):
                break

    def update_collision_constraints(self):
        for c in self.constraint_handler.constraints[:]: # iterating over a copy so that removing mid loop doesn't skip elements # TODO: find way of maintaining contacts that don't disappear
            if isinstance(c, constraint.ContactConstraint) or isinstance(c, constraint.FrictionConstraint):
                self.constraint_handler.remove_constraint(c)
        for c in self.correction_constraint_handler.constraints[:]: # iterating over a copy so that removing mid loop doesn't skip elements # TODO: find way of maintaining contacts that don't disappear
            if isinstance(c, constraint.ContactConstraint) or isinstance(c, constraint.FrictionConstraint):
                self.correction_constraint_handler.remove_constraint(c)

        self.collision_handler.update_collisions()

        for c in self.collision_handler.collisions:
            contact_constraint = constraint.ContactConstraint(c, impact_speed_threshold=self.constraint_handler.impact_speed_threshold)
            self.constraint_handler.add_constraint(contact_constraint)
            self.correction_constraint_handler.add_constraint(contact_constraint)

            friction_constraint = constraint.FrictionConstraint(c)
            self.constraint_handler.add_constraint(friction_constraint)

    def step(self, dt): # Chapter 4.1 details the steps in this method
        # steps 1 and 2 happen outside this method

        # integrate() consumes and zeroes each body's forces, but the frame's forces act over the
        # whole frame, so keep a copy to restore before every sub-step and every retry
        frame_wrenches = [m.rwrench.copy() for m in self.movables]

        remaining_dt = dt
        sub_dt = dt # carries over between sub-steps instead of retrying from remaining_dt every time
        while remaining_dt > 0:
            sub_dt = min(sub_dt, remaining_dt)

            # snapshot the state at the start of this subframe so a rejected attempt can be fully undone
            subframe_poses = [m.pose.copy() for m in self.movables]
            subframe_vels = [m.vel.copy() for m in self.movables]

            # halving sub_dt only helps when interpenetration is CAUSED by this subframe's integration; if it was
            # already there beforehand, no amount of halving fixes that (that's correct_drift's job), so skip straight to accepting
            prev_interpenetration = any(c.dist < 0 for c in self.collision_handler.collisions)

            while True:
                # restore the start-of-subframe state and forces before each attempt
                for i, m in enumerate(self.movables):
                    m.pose = subframe_poses[i].copy()
                    m.vel = subframe_vels[i].copy()
                    m.rwrench = frame_wrenches[i].copy()

                self.integrate(sub_dt) # steps 3, 4, 5 and 6 # TODO: previously inserted contact and friction constraints are only considered in the first iteration

                # step 7: rebuild the contact and friction constraints from the new positions
                self.update_collision_constraints()

                if prev_interpenetration:
                    break

                # check for interpenetration
                interpenetration = False
                for c in self.collision_handler.collisions:
                    if c.dist < 0:
                        interpenetration = True
                        break

                # accept the step if it is penetration-free, or once sub_dt has shrunk to the minimum;
                # below that we stop halving and let the contact solve and drift correction resolve the penetration
                if not interpenetration or sub_dt <= self.min_sub_dt:
                    break

                sub_dt /= 2

            remaining_dt -= sub_dt

            self.correct_drift() # steps 8 and 9
