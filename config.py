MIN_DISTANCE = 1.5 # minimum required distance (meters) between robot and person
GRID_SIZE = (10, 10) # size of grid

# initial positions
ROBOT_START = [0,0]
PEDESTRIAN_START = [0,1]
PEDESTRIAN_END = [4,5]

# time information
TIME_STEP = 0.1
SIMULATION_TIME = 20

# preset pedestrian paths
PEDESTRIAN_PATH1 = [(5,5),(4,5),(3,5),(2,5),(1,5),(0,5),(1,5)]
PEDESTRIAN_PATH2 = [(2, 0), (2, 1), (2, 2), (2, 3), (2, 4), (2, 5), (3, 5), (3, 4), (3, 3), (2, 3)]
PEDESTRIAN_PATH3 = [(4, 0), (3, 1), (2, 2), (1, 3), (0, 4), (1, 4), (2, 4), (3, 4), (4, 4), (5, 4)]
PEDESTRIAN_PATH4 = [(5, 0), (4, 1), (3, 2), (2, 3), (1, 4), (0, 5), (1, 6), (2, 7), (3, 8), (4, 9)]
PEDESTRIAN_PATH5 = [(3, 3), (3, 4), (4, 4), (4, 3), (3, 3), (2, 3), (2, 4), (1, 4), (1, 3), (0, 3)]
PEDESTRIAN_PATH6 = [(5, 0), (4, 1), (5, 2), (4, 3), (5, 4), (4, 5), (5, 6), (4, 7), (5, 8), (4, 9)]
PEDESTRIAN_PATH7 = [(1, 0), (1, 1), (1, 2), (1, 3), (1, 4), (1, 5), (1, 6), (1, 7), (1, 8), (1, 9)]
PEDESTRIAN_PATH8 = [(0, 5), (1, 5), (2, 5), (3, 5), (4, 5), (5, 5), (5, 5), (5, 5), (5, 5), (5, 5)]
PEDESTRIAN_PATH9 = [(2, 0), (3, 1), (4, 2), (5, 3), (5, 4), (5, 5), (4, 6), (3, 7), (2, 8), (1, 9)]
PEDESTRIAN_PATH10 = [(5, 0), (4, 1), (3, 2), (2, 3), (1, 4), (0, 5), (1, 6), (2, 7), (3, 8), (4, 9)]

PEDESTRIAN_PATH11 = [(5,5),(4,5),(3,5),(2,5),(1,5),(0,5)]

PED_PATH_REV = PEDESTRIAN_PATH11.copy()
PED_PATH_REV.reverse()

# set path for simulation
PEDESTRIAN_PATH = (PEDESTRIAN_PATH11 + PED_PATH_REV) * 5