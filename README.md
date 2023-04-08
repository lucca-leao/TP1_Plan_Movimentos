# motion
Some motion algorithms simulated in Stage ROS.

# Instructions
- Run:
```
roscore
```
- Open a new terminal and run:
```
cd scripts
```
- To start the simulation, open a new terminal and run :
```
rosrun stage_ros stageros -d worlds/<map_name>.world
```
- To run the Tangent Bug algorithm, open a new terminal and run:
```
python3 scripts/Q1_tangent_bug.py
```
and type in the (x,y) coordinates of the target.
- To run the Trajectory Following algorithm, open a new terminal and run:
```
python3 scripts/Q2_trajectory_following.py
```
- To run the Potential Field algorithm, open a new terminal and run:
```
python3 scripts/Q3_potential_field.py
```
- To run the Wavefront algorithm, open a new terminal and run:
```
python3 scripts/Q4_wavefront.py
```
