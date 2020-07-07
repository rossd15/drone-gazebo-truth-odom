# GAZEBO_TRUTH_ODOM PACKAGE


## Running and testing the nodes

1. Use catkin_make to build
```
cd ~/your_ros_ws/src
git clone https://github.com/rossd15/drone-gazebo-truth-odom.git
cd ..
catkin_make
```
2. Launch

```
cd ~/your_ros_ws
source devel/setup.bash
roslaunch gazebo_truth_odom drone_gazebo_truth_odom.launch
```
3. Gazeb ground truth topic `/odometry/truth` (for `iris/base_link)`
