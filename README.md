# Path Planning Decision Framework - README

## Overview

This project implements a decision framework for autonomous robots to navigate safely and efficiently around pedestrians in dynamic environments. The framework includes components such as pedestrian path prediction, continuous monitoring, interaction time analysis, and adaptive replanning to ensure smooth operation and collision avoidance.

## Requirements

- Python 3.x
- Required Python packages (install using `pip install -r requirements.txt` ):

## File Structure

- **`main.py`**: The main script to run the simulation. It integrates the entire decision framework and visualizes the robot's navigation in a shared space with pedestrians.
- **`config.py`**: Contains configurable parameters such as grid size, robot and pedestrian start positions, simulation time, and other heuristic-based parameters.

## Configuring Parameters

All configurable parameters are defined in the `config.py` file. Before running the simulation, you can adjust these parameters to fit your specific scenario:

- **Grid Size**: Defines the size of the simulation environment.
- **Start Positions**: Sets the initial positions of the robot and pedestrians.
- **Simulation Time**: Sets the total time for the simulation.
- **Pedestrian Path Number**: Adjust the paths to explore different situations.

## Running the Simulation

To run the simulation, simply execute the `main.py` script:

```bash
python main.py
