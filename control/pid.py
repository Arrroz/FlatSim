import numpy as np

class PIDController():

    def __init__(self, kp=1.0, ki=0, kd=0, out_min=None, out_max=None):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.out_min = out_min
        self.out_max = out_max

        self.ierror = None

    def update(self, dt, error, derror=None):
        error = np.asarray(error)        
        self.ierror = np.zeros(error.shape)
        if self.out_min is None: self.out_min = np.full(error.shape, float("-inf"))
        if self.out_max is None: self.out_max = np.full(error.shape, float("inf"))
        self.out_min = np.asarray(self.out_min)
        self.out_max = np.asarray(self.out_max)

        self.update = self._update_initialized
        return self._update_initialized(dt, error, derror)
    
    def _update_initialized(self, dt, error, derror=None):
        error = np.asarray(error)
        if derror is None: derror = np.zeros(error.shape)
        derror = np.asarray(derror)

        self.ierror += error * dt

        output = self.kp*error + self.ki*self.ierror + self.kd*derror

        # anti-windup
        for i in range(len(output)):
            if output[i] < self.out_min[i]:
                self.ierror[i] -= error[i] * dt
                output[i] = self.out_min[i]
            elif output[i] > self.out_max[i]:
                self.ierror[i] -= error[i] * dt
                output[i] = self.out_max[i]
        
        if len(output) == 1: return output[0]
        return output
    
