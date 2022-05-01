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

def draw_polynomial_path(ax: plt.Axes, T, poses: List[SE2], *args, **kwargs):
    velocity = []

    for i in range(len(poses) - 1):
        p0 = np.array([poses[i].x, poses[i].y])
        p1 = np.array([poses[i + 1].x, poses[i + 1].y])
        dp = p1 - p0 # Vector between start and goal poses
        dp = dp/np.linalg.norm(dp) # Unit vector of dp
        velocity.append(dp)

    velocity.append([0,0])

    trajx = local_planner_polynomial(points=[pose.x for pose in poses], T=T, v_list=[v[0] for v in velocity])
    trajy = local_planner_polynomial(points=[pose.y for pose in poses], T=T, v_list=[v[1] for v in velocity])
    
    ax.plot(trajx['x'], trajy['x'], label='trajectory', *args, **kwargs)