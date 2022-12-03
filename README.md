# Automated Biomimetic Fingertip Deformation Data Collection Using Computer Vision and Robotic Manipulation

#### Project completed by Cody Nichoson

### Project Description
This project served as a final project in Northwestern University's MS in Robotics sequence in 2022. The goal of the project was to produce an automated system capable of collecting biomimetic fingertip manipulation data and the fingertip deformation associated with it. The project is structured as a ROS2 package written in C++.

This project was advised by [Dr. Matthew Elwin](https://robotics.northwestern.edu/people/profiles/faculty/elwin-matt.html), [Dr. Ed Colgate](https://www.mccormick.northwestern.edu/research-faculty/directory/profiles/colgate-edward.html), and [Dr. Kevin Lynch](https://www.mccormick.northwestern.edu/research-faculty/directory/profiles/lynch-kevin.html).

### Hardware
* Franka Emika Panda Manipulator Arm
  * https://www.franka.de/
* Allied Vision Alvium 1800 U-500c, 1/2.5" 5.0MP C-Mount, USB 3.1 Color Camera
  * https://www.edmundoptics.com/p/allied-vision-alvium-1800-u-500c-125-50mp-c-mount-usb-31-color-camera/42324/

### General Software
* Ubuntu 22.04
  * https://releases.ubuntu.com/22.04/
* ROS2 Humble Hawksbill
  * https://docs.ros.org/en/humble/index.html
  
### Software Dependencies
* Allied Vision Vimba
* `avt_vimba_camera`
* `franka_ros2`
* `moveit` (MoveIt 2)
  * https://moveit.picknik.ai/humble/index.html
* `opencv2`
* `cv_bridge`

### Package Structure
The package is composed of several specialized ROS2 C++ packages:

* `finger_rig_description`
  * This package contains the Rviz configuration file for visualizing both the Franka arm and the computer vision aspects of the project.

* `finger_rig_bringup`
  * This package primarily contains the launch files necessary to run all of the necessary nodes, controllers, and configurations.

* `finger_rig_msgs`
  * This package contains a few custom service types used for the project.

* `finger_rig_control`
  * This package contains the bulk of the project in the form of multiple nodes for perception, motion control, and environmental configuration.
  
For more detailed information regarding the compnents of each of these packages, please see their individual README.md files.
