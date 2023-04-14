# Mini challenge 1 - Package

## Authors:
- Abril Berenice Bautista Roman
- Jose Angel Del Angel Dominguez
- Leonardo Javier Nava Castellanos
- Raul Lopez Musito

This packege includes nodes, services and other files to simulate the Manchester Robotics puzzlebot and to execute experiments with the real robot and simulation.

## Simulation files

| Simulation files |
|---------------------------|
| **Python nodes:**           |
| [- coordinateTransform.py](src/coordinateTransform.py)    |
| [- differentialDriveModel.py](src/differentialDriveModel.py) |
| [- solver.py](src/solver.py)                 |
| [- transform.py](src/transform.py)              |
| **Service files:**           |
| [- ResetPuzzlebotSim.srv](srv/ResetPuzzlebotSim.srv)  |
| **Launch files:**           |
| [- our_puzzlebot_sim.launch](launch/our_puzzlebot_sim.launch)  |
| **Other files:**           |
| [- meshes/*](meshes)                   |
| [- rviz_config/*](rviz_config)              |
| [- urdf/puzzlebot.xacro](urdf/puzzlebot.xacro)              |

## Navigation files

| Navigation files |
|---------------------------|
| **Python nodes:**           |
| [- odometry.py](src/odometry.py)    |
| [- point2PointController.py](src/point2PointController.py)    |
| **Service files:**           |
| [- ResetOdometry.srv](srv/ResetOdometry.srv)  |
| **Other files:**           |
| None                   |

## Experiments files

| Experiments files |
|---------------------------|
| **Python nodes:**           |
| [- our_sim_experiment_2.py](src/our_sim_experiment_2.py)    |
| [- generate_stats.py](experiments_data/generate_stats.py)    |
| **Other files:**           |
| [- experiments data](experiments_data/)                   |
