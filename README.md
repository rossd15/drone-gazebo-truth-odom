# GAZEBO_TRUTH_ODOM PACKAGE


## Running and testing the nodes

1. Use the launch file. Round 1 and 3, `{rover_type}` is `scout`. For round 2 `{rover_type}` is either `excavator` or `hauler`.

```
cd ~/srcp2-competitors/ros_workspace/
source devel/setup.bash
roslaunch gazebo_truth_odom {rover_type}_gazebo_truth_odom
```