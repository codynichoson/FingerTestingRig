# Automated Biomimetic Fingertip Deformation Data Collection Using Computer Vision and Robotic Manipulation

#### Project completed by Cody Nichoson

### Project Description
This project served as a final project in Northwestern University's MS in Robotics sequence in 2022. The goal of the project was to produce an automated system capable of collecting biomimetic fingertip manipulation data and the fingertip deformation associated with it. The project is structured as a ROS2 package written in C++.

This project was advised by [Dr. Matthew Elwin](https://robotics.northwestern.edu/people/profiles/faculty/elwin-matt.html), [Dr. Ed Colgate](https://www.mccormick.northwestern.edu/research-faculty/directory/profiles/colgate-edward.html), and [Dr. Kevin Lynch](https://www.mccormick.northwestern.edu/research-faculty/directory/profiles/lynch-kevin.html).

### Hardware
* <a href="https://www.franka.de/" target="_blank">Franka Emika Panda Manipulator Arm</a>
* <a href="https://www.edmundoptics.com/p/allied-vision-alvium-1800-u-500c-125-50mp-c-mount-usb-31-color-camera/42324/)" target="_blank">Allied Vision Alvium 1800 U-500c, 1/2.5" 5.0MP C-Mount, USB 3.1 Color Camera</a>

### General Software
* <a href="https://releases.ubuntu.com/22.04/" target="_blank">Ubuntu 22.04 Jammy Jellyfish</a>
* <a href="https://docs.ros.org/en/humble/index.html" target="_blank">ROS2 Humble Hawksbill</a>
  
### Software Dependencies
* [Allied Vision Vimba SDK 6.0.0](https://www.alliedvision.com/en/products/vimba-sdk/)
* [avt_vimba_camera](https://github.com/astuff/avt_vimba_camera/tree/ros2_master)
* [franka_ros2](https://github.com/frankaemika/franka_ros2)
* [MoveIt 2](https://moveit.picknik.ai/humble/index.html)
* [OpenCV 4.6.0](https://opencv.org/opencv-4-6-0/)
* [cv_bridge (from vision_opencv)](https://github.com/ros-perception/vision_opencv/tree/humble)

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
  
## Package Usage
### Install dependencies
#### Vimba SDK
To install Vimba, follow the installation instructions from Allied Vision's website (https://www.alliedvision.com/en/products/vimba-sdk/#c1497). Vimba version 6.0.0 for Ubuntu LTS 20.04 was used for this project (even though it was done on an Ubuntu 22.04 system).

### Create Workspace and Build Package
In order to use the package, it must be housed within the `src` directory of a colcon workspace alongside the `avt_vimba_camera` package.
 
 1. Create colcon workspace and change into `src` directory
 ```
 mkdir -p ~/fingerrig_ros2_ws/src
 cd ~/fingerrig_ros2_ws/src
 ```
 
 2. Clone `avt_vimba_camera` package into `fingerrig_ros2_ws/src`, switch it to the `ros2_master` branch, then return to the `src` folder
 ```
 git clone git@github.com:astuff/avt_vimba_camera.git
 cd avt_vimba_camera
 git switch ros2_master
 cd ..
 ```
 
 3. Clone this package into `fingerrig_ros2_ws/src`
 ```
 git clone git@github.com:codynichoson/FingerTestingRig.git
 ```
 
 4. Return to root of colcon workspace and build
 ```
 cd ..
 colcon build
 ```
