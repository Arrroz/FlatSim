from robot import link, joint

class Robot():

    def __init__(self, base: link.Link, joints: list[joint.Joint]):
        self.base = base
        self.joints = joints

    def set_state(self, x, y, theta, j_angles):
        self.base.x = x
        self.base.y = y
        self.base.theta = theta
        
        for i in range(len(self.joints)):
            self.joints[i].set_angle(j_angles[i])
    
    def links(self):
        links = [self.base]
        for j in self.joints:
            if not j.child in links:
                links.append(j.child)
        
        return links

    def integrate(self, dt):
        for l in self.links():
            l.integrate(dt)
        
