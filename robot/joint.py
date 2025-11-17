import numpy as np
from physics import utils, body, constraint

class Joint(constraint.Constraint):
    
    def __init__(self, parent: body.Body, anchor_parent, child: body.Body, anchor_child, offset=0, actuated=True):
        super().__init__()
        
        self.parent = parent
        self.anchor_parent = anchor_parent
        self.child = child
        self.anchor_child = anchor_child
        self.offset = offset
        self.actuated = actuated
        
        self.bodies = [parent, child]
        self.equality = True


class RJoint(Joint):

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)

        self.dimension = 2

        self.set_angle(0)

    def get_angle(self):
        return self.child.theta - self.parent.theta - self.offset

    def set_angle(self, angle):
        self.child.theta = self.parent.theta + self.offset + angle

        self.child.pos = (self.parent.pos +
                          utils.rotation_m(self.parent.theta) @ self.anchor_parent -
                          utils.rotation_m(self.child.theta) @ self.anchor_child)

    def J(self, body: body.Body):
        if body == self.parent:
            anchor_x, anchor_y = self.anchor_parent
            sign = 1
        elif body == self.child:
            anchor_x, anchor_y = self.anchor_child
            sign = -1
        else:
            return None

        return sign * np.block([np.eye(2),
                                utils.rotation_m(body.theta) @ np.array([[-anchor_y], [anchor_x]])])

    def e(self):
        parent_joint_pos = self.parent.pos + utils.rotation_m(self.parent.theta) @ self.anchor_parent
        child_joint_pos = self.child.pos + utils.rotation_m(self.child.theta) @ self.anchor_child

        return parent_joint_pos - child_joint_pos
    

class RollingContactJoint(Joint):

    def __init__(self, radius, normal, *args, **kwargs):
        super().__init__(*args, **kwargs)

        self.dimension = 2
        
        self.radius = radius
        self.normal = normal

    def get_angle(self):
        return self.child.theta - self.parent.theta - self.offset
    
    def set_angle(self, angle):
        self.angle = angle

        self.child.theta = self.parent.theta + np.arctan2(self.normal[1], self.normal[0]) + self.offset + angle

        self.child.pos = (self.parent.pos +
                          utils.rotation_m(self.parent.theta) @ self.anchor_parent -
                          utils.rotation_m(self.child.theta) @ self.anchor_child +
                          utils.rotation_m(self.parent.theta) * self.radius @ (
                              np.eye(2) + angle*np.array([[0, -1], [1, 0]])
                          ) @ self.normal)

    def J(self, body: body.Body):
        if body == self.parent:
            return np.block([
                np.eye(2),
                utils.rotation_m(self.bodies[0].theta) @ (
                    np.array([[0, -1], [1, 0]]) @ self.anchor_parent -
                    self.radius * (self.bodies[1].theta - self.bodies[0].theta) * self.normal
                )
            ])

        elif body == self.child:
            return -np.block([
                np.eye(2),
                (utils.rotation_m(self.bodies[1].theta) @ np.array([[-self.anchor_child[1]], [self.anchor_child[0]]]) -
                utils.rotation_m(self.bodies[0].theta) * self.radius @ np.array([[-self.normal[1]], [self.normal[0]]]))
            ])

        else:
            return None
        
    def e(self):
        parent_joint_pos = np.array([self.bodies[0].x, self.bodies[0].y]) + utils.rotation_m(self.bodies[0].theta) @ (
                                self.anchor_parent + self.radius * (
                                    np.eye(2) + (self.bodies[1].theta - self.bodies[0].theta) * np.array([[0, -1], [1, 0]])
                                ) @ self.normal
                            )
        
        child_joint_pos = np.array([self.bodies[1].x, self.bodies[1].y]) + utils.rotation_m(self.bodies[1].theta) @ self.anchor_child

        return parent_joint_pos - child_joint_pos

