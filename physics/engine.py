import numpy as np
from physics import body, solver, constraint, collision

class Engine():

    # TODO: there's redundancy in getting the bodies as an argument while most of them are already in the constraint_collection
    def __init__(self, bodies: list[body.Body], constraint_collection: constraint.ConstraintCollection, correction_constraint_collection: constraint.ConstraintCollection,
                 integration_solver: solver.Solver = solver.LemkeSolver(), drift_solver: solver.Solver = solver.LemkeSolver()):
        self.constraint_collection = constraint_collection
        self.correction_constraint_collection = correction_constraint_collection
        self.integration_solver = integration_solver
        self.drift_solver = drift_solver

        self.bodies = bodies
        self.movables = [b for b in self.bodies if b.movable]
        self.collision_handler = collision.CollisionHandler(self.bodies)

        self.reset_solver_matrices()

        self.n_friction_constraints = 0

    def reset_solver_matrices(self):
        mat = self.constraint_collection.matrices
        c_mat = self.correction_constraint_collection.matrices

        self.integration_solver.M = np.block([[mat.M, np.zeros((mat.M_dim, mat.C_dim))],
                                              [np.zeros((mat.C_dim, mat.M_dim + mat.C_dim))]])
        self.drift_solver.M = np.block([[c_mat.M, np.zeros((c_mat.M_dim, c_mat.C_dim))],
                                        [np.zeros((c_mat.C_dim, c_mat.M_dim + c_mat.C_dim))]])

        self.integration_solver.q = np.zeros((mat.M_dim + mat.C_dim,))
        self.drift_solver.q = np.zeros((c_mat.M_dim + c_mat.C_dim,))

        self.integration_solver.num_eqs = mat.M_dim + mat.n_equalities
        self.drift_solver.num_eqs = c_mat.M_dim + c_mat.n_equalities
    
    def integrate(self, dt):
        # update necessary constraint matrices
        self.constraint_collection.update_jacobians()
        self.constraint_collection.update_forces_vector()
        self.constraint_collection.update_constraint_constants()
        mat = self.constraint_collection.matrices

        # update solver matrices as needed
        solver = self.integration_solver
        #if solver.q.shape[0] != mat.M_dim + mat.C_dim: # TODO: reinsert this; currently, inserting this makes M not update and have wrong dimensions because of the way the E and miu matrices are added to M
        self.reset_solver_matrices()
        
        # extra rows and columns of solver.M come from weird friction constraints # TODO: this is sooooooo ugly *barf*
        E = np.zeros((2*self.n_friction_constraints, self.n_friction_constraints))
        for i in range(self.n_friction_constraints):
            E[2*i,i] = 1
            E[2*i+1,i] = 1
        miu = 2 * np.identity(self.n_friction_constraints) # TODO: where to get friction coefficient from?
        extra_column = np.zeros((mat.M_dim + mat.C_dim, self.n_friction_constraints))
        extra_column[(mat.M_dim + mat.C_dim - 2*self.n_friction_constraints):(mat.M_dim + mat.C_dim), :] = E
        extra_row = np.zeros((self.n_friction_constraints, mat.M_dim + mat.C_dim + self.n_friction_constraints))
        extra_row[:,(mat.M_dim + mat.C_dim - 3*self.n_friction_constraints):(mat.M_dim + mat.C_dim - 2*self.n_friction_constraints)] = miu
        extra_row[:,(mat.M_dim + mat.C_dim - 2*self.n_friction_constraints):(mat.M_dim + mat.C_dim)] = -np.transpose(E)

        solver.M[:mat.M_dim, mat.M_dim:] = -np.transpose(mat.J)
        solver.M[mat.M_dim:, :mat.M_dim] = mat.J
        solver.q[:mat.M_dim] = -(mat.M @ mat.dq + dt * mat.F)
        solver.q[mat.M_dim:] = mat.k

        solver.M = np.block([[solver.M, extra_column],
                             [extra_row]])
        solver.q = np.block([solver.q, np.zeros((self.n_friction_constraints,))])

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
        # update necessary constraint matrices
        self.correction_constraint_collection.update_jacobians()
        self.correction_constraint_collection.update_constraint_errors()
        mat = self.correction_constraint_collection.matrices

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
    
    def update_collision_constraints(self):
        for c in self.constraint_collection.constraints[:]: # iterating over a copy so that removing mid loop doesn't skip elements # TODO: find way of maintaining contacts that don't disappear
            if isinstance(c, constraint.ContactConstraint) or isinstance(c, constraint.FrictionConstraint):
                self.constraint_collection.remove_constraint(c)
        for c in self.correction_constraint_collection.constraints[:]: # iterating over a copy so that removing mid loop doesn't skip elements # TODO: find way of maintaining contacts that don't disappear
            if isinstance(c, constraint.ContactConstraint) or isinstance(c, constraint.FrictionConstraint):
                self.correction_constraint_collection.remove_constraint(c)
        
        self.collision_handler.update_collisions()

        for c in self.collision_handler.collisions: # TODO: the next loop needs to be separate from this one just so all the contact constraints are added before the friction ones; the construction of the matrices should be agnostic to this
            contact_constraint = constraint.ContactConstraint(c)
            self.constraint_collection.add_constraint(contact_constraint)
            self.correction_constraint_collection.add_constraint(constraint.ContactConstraint(c)) # TODO: should this be a copy of the constraint instead of the same one?

        for c in self.collision_handler.collisions:
            friction_constraint = constraint.FrictionConstraint(c)
            self.constraint_collection.add_constraint(friction_constraint)

        self.n_friction_constraints = len(self.collision_handler.collisions) # TODO: should this variable be a part of the constraint collection?

    def step(self, dt): # Chapter 4.1 details the steps in this method
        # steps 1 and 2 happen outside this method

        remaining_dt = dt
        while remaining_dt > 0:
            dt = remaining_dt
            
            interpenetration = True
            while interpenetration and dt > 1e-4: # last condition prevents dt from getting consistently too low in full penetration events, allowing the simulation to proceed # TODO: looky here!! another loose and empirically determined constant <3
                interpenetration = False

                self.integrate(dt) # steps 3, 4, 5 and 6 # TODO: previously inserted contact and friction constraints are only considered in the first iteration

                # step 7
                
                # reset contact and friction constraints
                self.update_collision_constraints()

                # check for interpenetration
                for c in self.collision_handler.collisions:
                    if c.dist < 0:
                        interpenetration = True
                        break
                
                # revert the integration and update the time step in case there is interpenetration
                if interpenetration:
                    for m in self.movables:
                        m.x -= dt * m.vx
                        m.y -= dt * m.vy
                        m.theta -= dt * m.w
                    dt /= 2

            remaining_dt -= dt
            
            self.correct_drift() # steps 8 and 9

        
