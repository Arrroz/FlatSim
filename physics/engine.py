import numpy as np
from physics import solver, constraint

class Engine():

    def __init__(self, constraint_collection: constraint.ConstraintCollection, correction_constraint_collection: constraint.ConstraintCollection, integration_solver: solver.Solver, drift_solver: solver.Solver):
        self.constraint_collection = constraint_collection
        self.correction_constraint_collection = correction_constraint_collection
        self.body_collection = constraint_collection.body_collection

        self.integration_solver = integration_solver
        self.drift_solver = drift_solver

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
        movables = self.body_collection.movables
        for i in range(len(movables)):
            movables[i].vx = mat.dq[3*i]
            movables[i].vy = mat.dq[3*i+1]
            movables[i].w = mat.dq[3*i+2]

            movables[i].x += dt * movables[i].vx
            movables[i].y += dt * movables[i].vy
            movables[i].theta += dt * movables[i].w

            movables[i].rfx = 0
            movables[i].rfy = 0
            movables[i].rtorque = 0

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
        movables = self.body_collection.movables
        for i in range(len(movables)):
            movables[i].x += delta_q[3*i]
            movables[i].y += delta_q[3*i+1]
            movables[i].theta += delta_q[3*i+2]
    
    def update_collision_constraints(self):
        for c in self.body_collection.contact_constraints:
            self.constraint_collection.remove_constraint(c) # TODO: find way of maintaining contacts that don't disappear
            self.correction_constraint_collection.remove_constraint(c) # TODO: find way of maintaining contacts that don't disappear
        for c in self.body_collection.friction_constraints:
            self.constraint_collection.remove_constraint(c) # TODO: find way of maintaining contacts that don't disappear
            self.correction_constraint_collection.remove_constraint(c) # TODO: find way of maintaining contacts that don't disappear
        
        self.body_collection.check_collisions()

        for c in self.body_collection.contact_constraints:
            self.constraint_collection.add_constraint(c)
            self.correction_constraint_collection.add_constraint(c)
        
        self.n_friction_constraints = 0 # TODO: should this variable be a part of the constraint collection?
        for c in self.body_collection.friction_constraints:
            self.constraint_collection.add_constraint(c)
            self.n_friction_constraints += 1

    def step(self, dt): # Chapter 4.1 details the steps in this method
        # steps 1 and 2 happen outside this method

        remaining_dt = dt
        while remaining_dt > 0:
            dt = remaining_dt
            
            interpenetration = True
            while interpenetration and dt > 1e-4: # last condition prevents dt from getting consistently too low in full penetration events, allowing the simulation to proceed # TODO: looky here!! another loose and empirically determined constant <3
                interpenetration = False

                self.integrate(dt) # steps 3, 4, 5 and 6 # TODO: previously inserted contact and friction constraints are only considered in the first iteration

                if hasattr(self.body_collection, 'collidables'): # step 7 # TODO: make this into a method
                    # reset contact and friction constraints
                    self.update_collision_constraints()

                    # check for interpenetration
                    for c in self.body_collection.collisions:
                        if c.dist < 0:
                            interpenetration = True
                            break
                
                # revert the integration and update the time step in case there is interpenetration
                if interpenetration:
                    for m in self.body_collection.movables:
                        m.x -= dt * m.vx
                        m.y -= dt * m.vy
                        m.theta -= dt * m.w
                    dt /= 2

            remaining_dt -= dt
            
            self.correct_drift() # steps 8 and 9

        
