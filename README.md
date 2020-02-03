# 3D-scanner-ROS

Note: My work is mainly in the branch of registration

## Dependencies:
Complete pack of ROS Melodic and some control staff:

```bash
sudo apt-get install ros-melodic-ros-control ros-melodic-ros-controllers
```

## Quick Start

Gazebo:
```bash
    roslaunch gazebo_ros empty_world.launch 
```

Spawn:
```bash
    roslaunch scanner scanner_control.launch
```

Rviz (errors will hide if run scanner_control.launch):
```bash
    roslaunch scanner scanner_rviz.launch 
```

Moving script:
```bash 
    ### Keep signs like here or else picture will be flipped
    rosrun scanner main.py -1.57 1.57
```

Example of hand by Moving Joints:
```bash
    rostopic pub /scanner/joint2_position_controller/command std_msgs/Float64 "data: -0.9"
```

TODO: 
  * Kinect drivers
  * Points cloud to RViz
  * Add rqt
  * 3D model building algorithm
