import numpy as np
import scipy.optimize as opt
import math

# Ensure MIN_DISTANCE is defined in your config file or set a default value here
try:
    from config import MIN_DISTANCE
except ImportError:
    MIN_DISTANCE = 1.0  # Set a default minimum distance if not defined in config

# Social Force Model parameters
A = 2.1
B = 0.3

def within_range(point1: list, point2: list, min_distance: float) -> bool:
    """
    Takes 2 points and determines if the distance between them is less than a specified minimum distance.

    Args:
        point1 (list): The first point, a list of coordinates [x1, y1, ...].
        point2 (list): The second point, a list of coordinates [x2, y2, ...].
        min_distance (float): The minimum distance to consider a collision.

    Returns:
        bool: True if the distance between the points is less than min_distance, False otherwise.
    """
    if len(point1) != len(point2):
        raise ValueError("Both points must have the same number of dimensions.")
    
    distance = math.sqrt(sum((a - b) ** 2 for a, b in zip(point1, point2)))
    return distance < min_distance

def find_collisions(predicted_path: list, robot_path: list, min_distance: float) -> dict:
    """
    Determine if robot and pedestrian are predicted to collide at any given point along the path.

    Args:
        predicted_path (list): Pedestrian prediction path line, a list of points [x, y, ...].
        robot_path (list): Robot path line, a list of points [x, y, ...].
        min_distance (float): The minimum distance to consider a collision.

    Returns:
        dict: A dictionary with the index as key and a tuple of points (robot_path[i], predicted_path[i]) as value where collisions are predicted to occur.
    """
    pois = {}
    
    for i in range(min(len(robot_path), len(predicted_path))):
        if within_range(robot_path[i], predicted_path[i], min_distance):
            pois[i] = (robot_path[i], predicted_path[i])
    
    return pois

def social_force(pedestrian_pos, robot_pos):
    """
    Calculate the repulsive force from the robot to the pedestrian using SFM.
    
    Args:
        pedestrian_pos (list): Position of the pedestrian [x, y].
        robot_pos (list): Position of the robot [x, y].

    Returns:
        np.array: Repulsive force vector [fx, fy].
    """
    diff = np.array(pedestrian_pos) - np.array(robot_pos)
    dist = np.linalg.norm(diff)
    force_magnitude = A * np.exp(-dist / B)
    force_vector = force_magnitude * (diff / dist)
    return force_vector

def optimize_trajectory(robot_pos, pedestrian_pos, min_distance):
    """
    Optimize the robot's trajectory to avoid collision with the pedestrian.

    Args:
        robot_pos (list): Current position of the robot [x, y].
        pedestrian_pos (list): Current position of the pedestrian [x, y].
        min_distance (float): The minimum distance to maintain from the pedestrian.

    Returns:
        np.array: The new position of the robot [x, y].
    """
    def objective_function(new_pos):
        return np.linalg.norm(new_pos - np.array(robot_pos))

    def constraint_function(new_pos):
        return np.linalg.norm(new_pos - np.array(pedestrian_pos)) - min_distance

    cons = {'type': 'ineq', 'fun': constraint_function}
    result = opt.minimize(objective_function, np.array(robot_pos), constraints=cons)
    
    if result.success:
        return result.x
    else:
        return np.array(robot_pos)

def avoid_collisions_with_sfm(robot_path, pedestrian_path, short_range_threshold=3.0):
    """
    Adjust the robot's path to avoid collisions using SFM for short-range interactions and optimization for long-range interactions.
    
    Args:
        robot_path (list): The original robot path, a list of points [x, y, ...].
        pedestrian_path (list): The pedestrian's predicted path, a list of points [x, y, ...].
        short_range_threshold (float): The distance threshold to switch from SFM to optimization-based avoidance.

    Returns:
        list: The adjusted robot path.
    """
    adjusted_path = []
    for i, (robot_pos, ped_pos) in enumerate(zip(robot_path, pedestrian_path)):
        force_from_pedestrian = social_force(ped_pos, robot_pos)
        dist_to_pedestrian = np.linalg.norm(np.array(robot_pos) - np.array(ped_pos))
        
        if dist_to_pedestrian < short_range_threshold:
            adjusted_pos = np.array(robot_pos) + force_from_pedestrian
        else:
            adjusted_pos = optimize_trajectory(robot_pos, ped_pos, min_distance)

        adjusted_path.append(adjusted_pos.tolist())
    return adjusted_path

# Example usage
robot_path = [[0, 0], [0, 1], [0, 2], [0, 3], [1, 3], [1, 2], [1, 1], [1, 0]]
pedestrian_path = [[0, 0], [1, 1], [2, 2], [0, 3], [2, 2], [2, 2], [1, 2], [1,0]]
min_distance = MIN_DISTANCE

pois = find_collisions(pedestrian_path, robot_path, min_distance)
adjusted_robot_path = avoid_collisions_with_sfm(robot_path, pedestrian_path, min_distance)
print("Points of Interaction (Collisions):", pois)
print("Adjusted Robot Path:", adjusted_robot_path)