# 3D-scanner-ROS

## Quick Start

Gazebo:

    roslaunch gazebo_ros empty_world.launch 
    
Spawn:

    roslaunch scanner scanner_control.launch

Rviz:

    roslaunch scanner scanner_rviz.launch 

Moving script:

    rosrun scanner main.py 1.57 -1.57


Example of hand by Moving Joints:

    rostopic pub /scanner/joint2_position_controller/command std_msgs/Float64 "data: -0.9"
    
TODO: 
  * Kinect drivers
  * Points cloud to RViz
  * Add rqt
  * 3D model building algorithm
