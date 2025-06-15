import numpy as np
from physics import utils
from robot import link

class Joint():
    
    def __init__(self, parent: link.Link, anchor_parent, child: link.Link, anchor_child, offset=0, actuated=True):
        self.parent = parent
        self.anchor_parent = anchor_parent

        self.child = child
        self.anchor_child = anchor_child

        self.offset = offset

        self.actuated = actuated
    

class RJoint(Joint):
    
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        
        self.set_angle(0)
    
    def get_angle(self):
        return self.child.theta - self.parent.theta - self.offset

    def set_angle(self, angle):
        self.angle = angle

        self.child.theta = self.parent.theta + self.offset + angle

        child_pos = (np.array([self.parent.x, self.parent.y]) +
                     np.matmul(utils.rotation_m(self.parent.theta), self.anchor_parent) -
                     np.matmul(utils.rotation_m(self.child.theta), self.anchor_child))
        
        self.child.x = child_pos[0]
        self.child.y = child_pos[1]


class RollingContactJoint(Joint): # only valid for circles rolling on a flat surface; no endpoints

    def __init__(self, radius, normal, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.radius = radius
        self.normal = normal
        
        self.set_angle(0)
        
    def get_angle(self):
        return self.child.theta - self.parent.theta - self.offset
    
    def set_angle(self, angle):
        self.angle = angle

        self.child.theta = self.parent.theta + np.arctan2(self.normal[1], self.normal[0]) + self.offset + angle

        child_pos = (np.array([self.parent.x, self.parent.y]) +
                     utils.rotation_m(self.parent.theta) @ self.anchor_parent -
                     utils.rotation_m(self.child.theta) @ self.anchor_child +
                     utils.rotation_m(self.parent.theta) * self.radius @ (
                         np.eye(2) + angle*np.array([[0, -1], [1, 0]])
                     ) @ self.normal)
        
        self.child.x = child_pos[0]
        self.child.y = child_pos[1]

