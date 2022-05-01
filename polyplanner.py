from typing import List
import numpy as np
from model import SE2

def local_planner_polynomial(poses: List[SE2]):
    t = []
    t0 = 0
    for i in range(len(poses)):
        t.append(t0 + 5 * i)

    for i in range(len(poses) - 1):
        start = poses[i]
        end = poses[i+1]

        A_x = np.array([
            [pow(t[i], 4), pow(t[i], 3), pow(t[i], 2), t[i], 1],
            [pow(t[i+1], 4), pow(t[i+1], 3), pow(t[i+1], 2), t[i+1], 1],
            [4*pow(t[i], 3), 3*pow(t[i], 2), 2*t[i], 1, 0],
            [4*pow(t[i+1], 3), 3*pow(t[i+1], 2), 2*t[i+1], 1, 0],
        ])
        B_x = np.array([[start.x],[end.x],[0],[0]])

        A_y = np.array([
            [pow(t[i], 4), pow(t[i], 3), pow(t[i], 2), t[i], 1],
            [pow(t[i+1], 4), pow(t[i+1], 3), pow(t[i+1], 2), t[i+1], 1],
            [4*pow(t[i], 3), 3*pow(t[i], 2), 2*t[i], 1, 0],
            [4*pow(t[i+1], 3), 3*pow(t[i+1], 2), 2*t[i+1], 1, 0],
        ])
        B_y = np.array([[start.y],[end.y],[0],[0]])

        coeff_x = np.linalg.solve(A_x, B_x)
        coeff_y = np.linalg.solve(A_y, B_y)
