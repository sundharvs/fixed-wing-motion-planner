from cProfile import label
from os import close
from turtle import distance
import numpy as np
from model import SE2, SE2_Tree, CircleObstacleArray, local_planner_straight_line
from view import draw_circle_obstacles, draw_polynomial_path, draw_se2, draw_se2_path, draw_se2_tree
import matplotlib.pyplot as plt


def main():
    obstacles = CircleObstacleArray.generate_uniform(x_range=(1,9), y_range=(1,9), r_range=(0.5, 1), samples=15)

    start = SE2(x=0, y=0, theta=0)
    goal = SE2(x=10, y=10, theta=1)

    root = SE2_Tree(start)
    node = root
    max_iter = 10000
    iter = 0

    while True:
        if iter > max_iter:
            print("Max iterations exceeded")
            break
        iter += 1

        end_goal = np.random.rand() < 0.1
        if end_goal:
            tmp_goal = goal
        else:
            tmp_goal = SE2.rand((0, 10), (0,10), (-np.pi, np.pi))
        
        d,closest = root.find_closest(tmp_goal)
        if end_goal and d < 1:
            break
        pose_new = local_planner_straight_line(
            start = closest.pose, goal = tmp_goal,
            distance = 1, dtheta_max = 0.5
        )
        if pose_new is None:
            continue
        close_i, close_d = obstacles.closest_obstacle(pose_new.x, pose_new.y)
        if close_d < 0:
            continue
        else:
            child = SE2_Tree(pose_new)
            closest.add_child(child)

    ax = plt.gca()
    h1 = draw_se2(ax, start, color='b', label='start')
    h2 = draw_se2(ax, goal, color='g', label='goal')
    h3 = draw_circle_obstacles(ax, obstacles, color='r', label='obstacles')[0]
    h4 = draw_se2_tree(ax, root, color='k', label='tree')[0]
    #h5 = draw_se2_path(ax, closest.path(), color='g', label='path', linewidth=3, linestyle='dashed')
    h5 = draw_polynomial_path(ax, closest.path(), color='g', label='poly path', linewidth=3)[0][0]
    plt.xlabel('x, m')
    plt.ylabel('y, m')
    plt.legend(handles=[h1, h2, h3, h4, h5], loc='upper left', ncol=2)
    plt.title('RRT Motion Planning')
    plt.axis([-2, 12, -2, 12])
    plt.grid()
    plt.savefig("test.png")
    plt.close()

main()