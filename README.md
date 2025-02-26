# px4-noetic-gazebo

A repository of running multi-quadrotor simulations in Gazebo with PX4 inside Docker container.

## Requirements

* Docker

## Running commands

```bash
# in root directory
chmod +x docker/run_docker.sh
./docker/run_ros_docker.sh

# run all the commands
tmuxinator start . -p scene=two_obstacles

# to exit tmuxinator, press `Ctrl+b` and then `q` in any pane

# plot results
python3 src/multi_uav_formation/multi_uav_formation/PlotSingleRun.py
```