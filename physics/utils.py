import numpy as np

gravity = np.array([0, -9.81])

def rotation_m(angle):
    return np.array([[np.cos(angle), -np.sin(angle)],
                     [np.sin(angle), np.cos(angle)]])

def normalize(v):
    n = np.linalg.norm(v)
    if n == 0:
        return v
    return v/n
