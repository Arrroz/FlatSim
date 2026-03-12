import numpy as np

class Path():

    def __init__(self, path_function, max_step=0.001, dist_tolerance=0.1):
        self.dist_tolerance = dist_tolerance

        num_points = int(np.ceil(1/max_step))
        points = np.linspace(0.0, 1.0, num_points)

        self.path = [np.asarray(path_function(p)) for p in points]
        self.curr_point = 0

    def get_target(self, pos):
        while (np.linalg.norm(pos - self.path[self.curr_point]) < self.dist_tolerance
               and self.curr_point < len(self.path) - 1):
            self.curr_point += 1
        
        return self.path[self.curr_point]
    
    def end_reached(self):
        return self.curr_point == len(self.path) - 1
