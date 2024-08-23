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