from typing import List
from model import SE2_Tree, SE2, CircleObstacleArray
import matplotlib.pyplot as plt
from matplotlib.collections import PatchCollection
from matplotlib.patches import Patch
import numpy as np

def draw_circle_obstacles(ax: plt.Axes, obstacles: CircleObstacleArray, *args, **kwargs):
    col = PatchCollection([plt.Circle((x, y), radius) for x, y, radius in obstacles.data], *args, **kwargs)
    ax.add_collection(col)

    return Patch(*args, **kwargs), col

def draw_se2(ax: plt.Axes, pose: SE2, *args, **kwargs):
    return ax.quiver(pose.x, pose.y, np.cos(pose.theta), np.sin(pose.theta), *args, **kwargs)

def draw_se2_path(ax: plt.Axes, poses: List[SE2], *args, **kwargs):
    return ax.plot([pose.x for pose in poses], [pose.y for pose in poses], *args, **kwargs)[0]

def draw_se2_tree(ax: plt.Axes, node: SE2_Tree, *args, **kwargs):
    h = None
    for child in node.children:
        h = ax.plot([node.pose.x, child.pose.x], [node.pose.y, child.pose.y], *args, **kwargs)
        draw_se2_tree(ax, child, *args, **kwargs)
        return h

def draw_polynomial_path(ax: plt.Axes, poses:List[SE2], *args, **kwargs):
    t = []
    t0 = 0
    h = []
    for i in range(len(poses)):
        t.append(t0 + 1 * i)

    for i in range(len(poses) - 1):
        start = poses[i]
        end = poses[i+1]

        A = np.array([
            [pow(t[i], 5), pow(t[i], 4), pow(t[i], 3), pow(t[i], 2), t[i], 1],
            [pow(t[i+1], 5), pow(t[i+1], 4), pow(t[i+1], 3), pow(t[i+1], 2), t[i+1], 1],
            [5*pow(t[i], 4), 4*pow(t[i], 3), 3*pow(t[i], 2), 2*t[i], 1, 0],
            [5*pow(t[i+1], 4), 4*pow(t[i+1], 3), 3*pow(t[i+1], 2), 2*t[i+1], 1, 0],
            [20*pow(t[i], 3), 12*pow(t[i], 2), 6*t[i], 2, 0 ,0],
            [20*pow(t[i+1], 3), 12*pow(t[i+1], 2), 6*t[i + 1], 2, 0, 0]
        ])
        B_x = np.array([[start.x],[end.x],[np.cos(start.theta)],[np.cos(end.theta)],[0],[0]])
        B_y = np.array([[start.y],[end.y],[np.sin(start.theta)],[np.sin(end.theta)],[0],[0]])

        coeff_x = np.linalg.solve(A, B_x)
        coeff_y = np.linalg.solve(A, B_y)

        t_space = np.linspace(t[i], t[i + 1])
        x = np.polyval(coeff_x, t_space)
        y = np.polyval(coeff_y, t_space)

        h_c = ax.plot(x, y, *args, **kwargs)
        h.append(h_c)
    return h
