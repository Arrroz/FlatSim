import numpy as np
from physics import body, collision, utils
from robot import joint

class Constraint():

    def __init__(self):
        self.dimension = 0
        self.bodies = []
        self.equality = True # False if it is an inequality constraint (joints vs contacts)
    
    def J(self, body: body.Body):
        pass

    def e(self):
        pass

    def k(self):
        return np.zeros((self.dimension,))


class DataMatrices(): # used in the ConstraintCollection class to hold the matrices used by the solver; the only point it to better organize the ConstraintCollection class
    def __init__(self):
        self.M_dim = self.C_dim = self.n_equalities = 0
        self.M = self.F = self.dq = self.J = self.k = self.e = None


class ConstraintCollection():
    
    def __init__(self, constraints: list[Constraint], body_collection: body.BodyCollection):
        self.constraints = constraints
        self.body_collection = body_collection
        self.matrices = DataMatrices()

        self.reset_matrices()

    def reset_matrices(self):
        mat = self.matrices
        mat.M_dim = 3 * len(self.body_collection.movables)
        mat.C_dim = sum([c.dimension for c in self.constraints])
        mat.n_equalities = sum([c.dimension for c in self.constraints if c.equality])

        mat.M = np.diag(np.array([[b.mass, b.mass, b.moi] for b in self.body_collection.movables]).flatten()) # TODO: if Lagrangian dynamics are implemented, this matrix will have to be updated more often
        mat.F = np.zeros((mat.M_dim,), dtype=float)
        mat.dq = np.zeros((mat.M_dim,), dtype=float)

        mat.J = np.zeros((mat.C_dim, mat.M_dim), dtype=float)
        mat.k = np.zeros((mat.C_dim,), dtype=float)
        mat.e = np.zeros((mat.C_dim,), dtype=float)
        
    def update_forces_vector(self):
        start_i = 0
        for b in self.body_collection.movables:
            end_i = start_i + 3
            self.matrices.F[start_i:end_i] = [b.rfx, b.rfy, b.rtorque]
            start_i = end_i

    def update_jacobians(self):
        start_i = 0
        for c in self.constraints:
            end_i = start_i + c.dimension
            
            start_j = 0
            for b in self.body_collection.movables:
                end_j = start_j + 3

                if b in c.bodies:
                    self.matrices.J[start_i:end_i, start_j:end_j] = c.J(b)
            
                start_j = end_j

            start_i = end_i

    def update_constraint_constants(self):
        start_i = 0
        for c in self.constraints:
            end_i = start_i + c.dimension
            self.matrices.k[start_i:end_i] = c.k()
            start_i = end_i

    def update_constraint_errors(self):
        start_i = 0
        for c in self.constraints:
            end_i = start_i + c.dimension
            self.matrices.e[start_i:end_i] = c.e()
            start_i = end_i
    
    def add_constraint(self, constraint: Constraint):
        self.constraints.append(constraint)
        mat = self.matrices

        mat.C_dim += constraint.dimension
        if constraint.equality:
            mat.n_equalities += constraint.dimension

        mat.J = np.block([[mat.J], [np.zeros((constraint.dimension, mat.M_dim))]])
        mat.k = np.block([mat.k, np.zeros((constraint.dimension,))])
        mat.e = np.block([mat.e, np.zeros((constraint.dimension,))])
        
    def remove_constraint(self, constraint: Constraint):
        if not constraint in self.constraints:
            return
        
        # find index of constraint in the matrices
        i = 0
        for c in self.constraints:
            if c == constraint:
                break
            i += c.dimension
        
        self.constraints.remove(constraint)
        mat = self.matrices

        mat.C_dim -= constraint.dimension
        if constraint.equality:
            mat.n_equalities -= constraint.dimension
        
        mat.J = np.delete(mat.J, [i, i+constraint.dimension-1], 0)
        mat.k = np.delete(mat.k, [i, i+constraint.dimension-1], 0)
        mat.e = np.delete(mat.e, [i, i+constraint.dimension-1], 0)

    def add_body(self, body: Constraint):
        pass # TODO
    
    def remove_body(self, body: Constraint):
        pass # TODO


class RJointConstraint(Constraint):

    def __init__(self, joint: joint.RJoint):
        self.joint = joint
        self.dimension = 2
        self.bodies = [self.joint.parent, self.joint.child]
        self.equality = True

    def J(self, body: body.Body):
        if body == self.joint.parent:
            anchor_x, anchor_y = self.joint.anchor_parent
            sign = 1
        elif body == self.joint.child:
            anchor_x, anchor_y = self.joint.anchor_child
            sign = -1
        else:
            return None

        return sign * np.block([np.eye(2),
                                np.matmul(
                                    utils.rotation_m(body.theta),
                                    np.array([[-anchor_y], [anchor_x]]))])

    def e(self):
        parent = self.joint.parent
        child = self.joint.child

        parent_joint_pos = np.array([parent.x, parent.y]) + np.matmul(utils.rotation_m(parent.theta), self.joint.anchor_parent)
        child_joint_pos = np.array([child.x, child.y]) + np.matmul(utils.rotation_m(child.theta), self.joint.anchor_child)

        return parent_joint_pos - child_joint_pos


class ContactConstraint(Constraint):

    def __init__(self, collision: collision.Collision):
        self.collision = collision
        self.dimension = 1
        self.bodies = [self.collision.collidable1, self.collision.collidable2]
        self.equality = False

    def J(self, body: body.Body):
        if body == self.bodies[0]:
            relative_pos = self.collision.relative_pos1
            sign = -1
        elif body == self.bodies[1]:
            relative_pos = self.collision.relative_pos2
            sign = 1
        else:
            return None

        return sign * np.block([self.collision.normal,
                                np.cross(relative_pos, self.collision.normal)])

    def e(self):
        return self.collision.dist
    
    def k(self):
        b1 = self.bodies[0]
        b2 = self.bodies[1]
        normal_relative_speed = np.matmul(self.J(b1), np.array([b1.vx, b1.vy, b1.w])) + np.matmul(self.J(b2), np.array([b2.vx, b2.vy, b2.w]))
        
        if -normal_relative_speed < 5: # TODO: what should this constant be?
            return super().k() # contact
        return b1.restitution * b2.restitution * normal_relative_speed # impact # TODO: is this how restitution of a collision be calculated?


class FrictionConstraint(Constraint):

    def __init__(self, collision: collision.Collision):
        self.collision = collision
        self.dimension = 2
        self.bodies = [self.collision.collidable1, self.collision.collidable2]
        self.equality = False

    def J(self, body: body.Body):
        if body == self.bodies[0]:
            relative_pos = self.collision.relative_pos1
            sign = -1
        elif body == self.bodies[1]:
            relative_pos = self.collision.relative_pos2
            sign = 1
        else:
            return None
        
        tangent = np.array([-self.collision.normal[1], self.collision.normal[0]])
        rotJ = np.cross(relative_pos, tangent)

        return sign * np.block([[tangent, rotJ],
                                [-tangent, -rotJ]])
    
    # TODO: should this be zeros?
    def e(self):
        return self.collision.dist # the extra is added to make sure contacts remain dispite numerical errors # TODO: how much should it be?
