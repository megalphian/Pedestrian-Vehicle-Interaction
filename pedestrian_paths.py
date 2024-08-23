import numpy as np
from config import *

def robot_path(grid_size):
    """
    Defines path of robot (lawnmower-style) in gridsize
    """
    path = []
    for i in range(grid_size[0]):
        if i % 2 == 0:
            # Move right
            for j in range(grid_size[1]):
                path.append((i, j))
        else:
            # Move left
            for j in range(grid_size[1] - 1, -1, -1):
                path.append((i, j))
    return path

def pedestrian_path(start, end, num_steps, robot_path, max_intersections=3):
    """
    Defines a pedestrian path that intersects the robot's path at least once and no more than three times.

    Args:
        start (tuple): Starting coordinates of the pedestrian (x, y).
        end (tuple): Ending coordinates of the pedestrian (x, y).
        num_steps (int): Number of steps for the pedestrian path.
        robot_path (list): List of coordinates representing the robot's path.
        max_intersections (int): Maximum number of intersections with the robot's path.

    Returns:
        list: Pedestrian path coordinates.
    """
    path = [start]
    current_pos = start
    steps_remaining = num_steps - 1

    # Ensure at least one and no more than max_intersections
    num_intersections = np.random.randint(1, max_intersections + 1)
    intersection_indices = np.random.choice(range(1, num_steps - 1), num_intersections, replace=False)

    for step in range(1, num_steps):
        if step in intersection_indices:
            next_pos = robot_path[step]
        else:
            # Move one step towards the end
            next_pos = (
                current_pos[0] + np.sign(end[0] - current_pos[0]),
                current_pos[1] + np.sign(end[1] - current_pos[1])
            )
        path.append(next_pos)
        current_pos = next_pos

    return path

# Example usage
GRID_SIZE = (10, 10)
robot_pat = robot_path(GRID_SIZE)
pedestrian_start = (1, 0)
pedestrian_end = (9,9)
num_steps = len(robot_pat)

ped_path = pedestrian_path(pedestrian_start, pedestrian_end, num_steps, robot_pat)
ped_path = [(5,5),(4,5),(3,5),(2,5),(1,5),(0,5),(1,5)]
print("Pedestrian Path:", ped_path)
print("Robot Path:", robot_pat)

for i in range(len(ped_path)):
    if ped_path[i] == robot_pat[i]:
        print(ped_path[i])

