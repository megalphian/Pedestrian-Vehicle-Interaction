import numpy as np

class PedestrianPrediction:
    def __init__(self, pedestrian_start):
        self.pedestrian_start = pedestrian_start

    def predict_next_positions(self, previous_positions,  num_steps_ahead):
        if len(previous_positions) < 2:
            return [previous_positions[-1]] * num_steps_ahead

        current_position = np.array(previous_positions[-1])
        previous_position = np.array(previous_positions[-2])
        velocity = current_position - previous_position

        # If zero velocity, return the current position for the next num_steps_ahead steps
        if velocity.all() == 0:
            return [tuple(current_position)] * num_steps_ahead
        
        desired_direction = velocity / np.linalg.norm(np.array(velocity))
        
        # MR: I am not sure about the following line
        acceleration = desired_direction - velocity
        
        predicted_positions = []
        for step in range(1, num_steps_ahead + 1):
            next_position = current_position + velocity * step + 0.5 * acceleration * (step) ** 2
            predicted_positions.append(tuple(next_position))
        return predicted_positions  

    def heuristic_predict_next_position(self, current_position, heading, speed, time_step):
        # Heuristic-based prediction focusing on collision avoidance and trajectory adherence
        # Adjust the heading slightly towards the end point while maintaining the speed
        target_direction = np.array(self.pedestrian_end) - np.array(current_position)
        target_direction = target_direction / np.linalg.norm(target_direction)
        
        # Simple heuristic to adjust heading and speed
        new_heading = (heading + target_direction) / 2  # Average direction towards target
        new_heading = new_heading / np.linalg.norm(new_heading)  # Normalize
        
        # Calculate the next position based on the adjusted heading and constant speed
        next_position = np.array(current_position) + new_heading * speed * time_step
        return tuple(next_position)