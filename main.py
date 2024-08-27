import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from matplotlib.lines import Line2D
from matplotlib.widgets import Button
from path_prediction import PedestrianPrediction
from pedestrian_paths import pedestrian_path
from robot_paths import robot_path
from config import *

# Environment size
grid_size = GRID_SIZE

# Initial positions
robot_position = ROBOT_START

# Simulation time
simulation_time = SIMULATION_TIME

# Initialize robot and pedestrian routes
robot_route = robot_path(grid_size)
num_steps = len(robot_route)
pedestrian_route = PEDESTRIAN_PATH
pedestrian_start = pedestrian_route[0]
pedestrian_end = pedestrian_route[-1]

# Pad the end of the pedestrian route to match the robot route length
pedestrian_route.extend([pedestrian_route[-1]] * (len(robot_route) - len(pedestrian_route)))

# Initialize array to store predictions, updated robot positions
predicted_positions = []
robot_positions = []

# Initialize prediction
predictor = PedestrianPrediction(pedestrian_start)

# Heuristic-based parameters
speed = 10.0  # Assuming constant speed for simplicity

# Key Terms and Definitions
# Beta: Maximum duration of pedestrian path predictions
# Alpha: Minimum distance required between robot and pedestrian
# Tau: Maximum allowable interaction time

# Number of steps ahead for prediction (Beta)
beta = 10

# Minimum safe distance (Alpha)
alpha = 5  # Threshold for replanning time

# Allowed Interaction Time (Tau)
tau = 3  # Threshold for different/consecutive interactions

# Function to check if two coordinates are the same
def coordinates_match(coord1, coord2):
    return np.allclose(coord1, coord2)

# Function to calculate Time to Interaction (TTI)
def time_to_interaction(robot_pos, pedestrian_pos, speed):
    return np.linalg.norm(np.array(robot_pos) - np.array(pedestrian_pos)) / speed

# Initialize a variable to track how long the pedestrian has stayed in the same position
stagnant_steps = 0

# Simulation loop
robot_step = 0
for sim_step in range(num_steps):
    if sim_step >= 2:  # Start predictions after initial steps
        if sim_step < 10:
            previous_positions = pedestrian_route[:sim_step+1]  # Use however many accumulated positions
        else:
            previous_positions = pedestrian_route[sim_step-9:sim_step+1]  # Use the last 10 positions

        # if step == 2 or step % 10 == 0:  # Initial prediction and every 10 steps
        #     future_positions = predictor.predict_next_positions(previous_positions, beta)
        # else:
        #     future_positions = predictor.predict_next_positions(previous_positions, 1) + future_positions[1:]

        future_positions = predictor.predict_next_positions(previous_positions, beta)

        predicted_positions.append(future_positions)
    else:
        predicted_positions.append([pedestrian_route[sim_step]])

    # Check if the pedestrian is stagnant (i.e., hasn't moved)
    if sim_step > 0 and coordinates_match(pedestrian_route[sim_step], pedestrian_route[sim_step - 1]):
        stagnant_steps += 1
    else:
        stagnant_steps = 0  # Reset if the pedestrian moves

    # Continuous Monitoring and Safety Check: Ensure the robot never occupies the same position as the pedestrian
    if coordinates_match(robot_route[robot_step], pedestrian_route[sim_step]):
        print(f"Stop at step {sim_step}: Robot and pedestrian would collide at {robot_route[sim_step]}")
        robot_positions.append(robot_positions[-1])  # Stop the robot (stay in place)
    else:
        interaction_time = time_to_interaction(robot_route[sim_step], pedestrian_route[sim_step], speed)

        # Interaction Time Analysis: Check if a replan is needed
        if interaction_time > alpha or stagnant_steps > tau:
            print(f"Replan at step {sim_step}: Pedestrian at {pedestrian_route[sim_step]} (stagnant for {stagnant_steps} steps)")
            # Start replan procedure
            robot_positions.append(robot_route[robot_step])  # Append current position before replanning
            break  # Add logic for replanning here
        else:
            robot_positions.append(robot_route[robot_step])  # Continue to the next position

        robot_step += 1
print("ped_route: ", pedestrian_route)
print("Robot_positions: ", robot_positions)

for i in range(len(pedestrian_route)):
    if robot_route[i] == pedestrian_route[i]:
        print(pedestrian_route[i])

# Visualization
fig, ax = plt.subplots()
ax.set_xlim(-1, grid_size[0] + 1)
ax.set_ylim(-1, grid_size[1] + 1)
ax.set_title("Robot and Pedestrian Simulation with Predictions")
ax.set_xlabel("X Position")
ax.set_ylabel("Y Position")
ax.grid(True)

robot_dot, = ax.plot([], [], 'bo', label='Robot')
pedestrian_dot, = ax.plot([], [], 'ro', label='Pedestrian')
prediction_line = Line2D([], [], color='green', label='Prediction')
ax.add_line(prediction_line)
robot_path_line = Line2D([], [], color='blue', linestyle='--', label='Robot Path')
ax.add_line(robot_path_line)

# Add a legend
ax.legend(loc='upper right')

# Pause/resume functionality
is_paused = False
frame = 0

def toggle_pause(event):
    global is_paused
    is_paused = not is_paused

pause_ax = plt.axes([0.81, 0.01, 0.1, 0.075])
pause_button = Button(pause_ax, 'Pause/Resume')
pause_button.on_clicked(toggle_pause)

def init():
    robot_dot.set_data([], [])
    pedestrian_dot.set_data([], [])
    prediction_line.set_data([], [])
    robot_path_line.set_data([], [])
    return [robot_dot, pedestrian_dot, prediction_line, robot_path_line]

def update(_):
    global frame
    if not is_paused:
        # Calculate the interpolation factor
        t = frame % 10 / 10.0

        # Safeguard to prevent out-of-range index access
        if frame // 10 >= len(robot_positions):
            return [robot_dot, pedestrian_dot, prediction_line, robot_path_line]

        # Interpolate between the current and next positions
        x_robot = (1 - t) * robot_positions[frame // 10][0] + t * robot_positions[min(frame // 10 + 1, len(robot_positions) - 1)][0]
        y_robot = (1 - t) * robot_positions[frame // 10][1] + t * robot_positions[min(frame // 10 + 1, len(robot_positions) - 1)][1]

        x_pedestrian = (1 - t) * pedestrian_route[frame // 10][0] + t * pedestrian_route[min(frame // 10 + 1, len(pedestrian_route) - 1)][0]
        y_pedestrian = (1 - t) * pedestrian_route[frame // 10][1] + t * pedestrian_route[min(frame // 10 + 1, len(pedestrian_route) - 1)][1]

        # Set the data with lists, even if they are single points
        robot_dot.set_data([x_robot], [y_robot])
        pedestrian_dot.set_data([x_pedestrian], [y_pedestrian])

        if frame // 10 < len(predicted_positions) and len(predicted_positions[frame // 10]) == beta:
            prediction_line.set_data(
                [predicted_positions[frame // 10][i][0] for i in range(beta)],
                [predicted_positions[frame // 10][i][1] for i in range(beta)]
            )

        # Update the robot path line
        robot_path_line.set_data([pos[0] for pos in robot_positions[:frame // 10 + 1]],
                                 [pos[1] for pos in robot_positions[:frame // 10 + 1]])

        frame += 1
    return [robot_dot, pedestrian_dot, prediction_line, robot_path_line]

ani = FuncAnimation(fig, update, frames=num_steps * 10, init_func=init, blit=True, repeat=False, interval=50)
plt.show()