
# Obstacle-Free Path Planning for Fixed-Wing Aircraft

This project generates obstacle-free paths designed to ensure smooth transitions in vehicle heading between waypoints, making them suitable for fixed-wing aircraft.

https://github.com/user-attachments/assets/2876521a-5614-4d9a-b6c1-271e2ac2d2cc

## Features

- **RRT Planning**: Implements the [Rapidly-exploring Random Trees (RRT)](https://en.wikipedia.org/wiki/Rapidly_exploring_random_tree) algorithm to explore the state space and find feasible paths from start to goal, avoiding obstacles.
- **SE2 Poses**: Uses [SE2](https://www.mathworks.com/help/robotics/ref/se2.html) (2D position + heading) poses to represent aircraft states.
- **Polynomial Trajectories**: Creates smooth, feasible paths between waypoints using fifth-order polynomial trajectories.

## Files

- **`test.py`**: Entry point for running the program. Contains an example of a problem setup, including start, goal, and obstacles.
- **`view.py`**: Contains the plotting tools for visualizing SE2 poses, obstacles, and the resulting polynomial path.
- **`model.py`**: Implements the core functionality of the project:
  - SE2 pose representation
  - A tree of SE2 poses for path planning
  - The obstacle map
  - RRT algorithm

## Usage

1. **Running the Example**:
   ```bash
   python test.py
   ```

2. **Modifying the Setup**:
   You can modify the start, goal, and obstacle locations by editing the corresponding variables in `test.py`:
   ```python
   start = SE2(x_start, y_start, theta_start)
   goal = SE2(x_goal, y_goal, theta_goal)
   obstacles = CircleObstacleArray.generate_uniform(x_range=(1,9), y_range=(1,9), r_range=(0.5, 1), samples=15)
   ```

## Dependencies

- `numpy`
- `matplotlib`
