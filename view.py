from typing import List
from model import SE2_Tree, SE2, CircleObstacleArray
from polyplanner import local_planner_polynomial
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

def draw_polynomial_path(ax: plt.Axes, poses:List[SE2]):
    vel = 1
    #waypoints = np.array([[pose.x, pose.y, vel*np.cos(pose.theta), vel*np.sin(pose.theta)] for pose in poses])
    waypoints = np.array([
    # px, py, vx, vy
    [0, 0, 1, -1],
    [1, 0, 1, 1],
    [1, 1, -1, 1],
    [0, 1, -1, -1],
    [0, 0, 1, -1]
    ])

    T = [1, 1, 1, 1]
    trajx = local_planner_polynomial(x_list=waypoints[:, 0], v_list=waypoints[:, 2], T=T, plot=True)
    trajy = local_planner_polynomial(x_list=waypoints[:, 1], v_list=waypoints[:, 3], T=T, plot=True)
    
    ax.plot(trajx['x'], trajy['x'], label='trajectory')

    return locals()