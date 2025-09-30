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
        self.child.theta = self.parent.theta + self.offset + angle

        self.child.pos = (self.parent.pos +
                          utils.rotation_m(self.parent.theta) @ self.anchor_parent -
                          utils.rotation_m(self.child.theta) @ self.anchor_child)
