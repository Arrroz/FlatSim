import numpy as np

class Transform():
    
    def __init__(self, translation=np.zeros((2,)), angle=0.0):
        self.translation = translation
        self.angle = angle
        
    @property
    def angle(self):
        return np.arctan2(self.rot_mat[0,1], self.rot_mat[0,0])
    @angle.setter
    def angle(self, value):
        self.rot_mat = np.array([[np.cos(value), np.sin(value)],
                                 [-np.sin(value), np.cos(value)]])
        
    def __invert__(self):
        translation = - self.rot_mat.T @ self.translation
        angle = -self.angle
        return Transform(translation, angle)

    def __matmul__(self, other):
        if not isinstance(other, np.ndarray):
            raise TypeError(f"Objects of type {other.__class__.__name} cannot be transformed using @")
        
        return self.rot_mat @ other + self.translation

    def __mul__(self, other):
        if not isinstance(other, Transform):
            raise TypeError(f"Objects of type {other.__class__.__name} cannot be transformed using *")
        
        translation = self @ other.translation
        angle = self.angle + other.angle
        return Transform(translation, angle)
    
    def __truediv__(self, other):
        return self * (~other)
    
    def __imul__(self, other):
        return self * other
    
    def __itruediv__(self, other):
        return self / other
        
    def derivative(self, var, var_frame): # self and var_frame must be relative to the same frame
        match var:
            case "x":
                return np.append(var_frame.rot_mat
                                 @ np.array([1, 0]),
                                 0)

            case "y":
                return np.append(var_frame.rot_mat
                                 @ np.array([0, 1]),
                                 0)

            case "angle":
                return np.append(var_frame.rot_mat
                                 @ np.array([[0, -1], [1, 0]])
                                 @ -(~var_frame * self).translation,
                                 1)

            case _:
                raise ValueError("Invalid variable: valid variables are 'x', 'y' and 'angle'")
