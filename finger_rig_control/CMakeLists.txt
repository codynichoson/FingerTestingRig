cmake_minimum_required(VERSION 3.8)
project(finger_rig_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

set(THIS_PACKAGE_INCLUDE_DEPENDS
  rclcpp
  std_msgs
  sensor_msgs
  cv_bridge
  OpenCV
  std_srvs
  moveit_ros_planning_interface
  moveit_msgs
  finger_rig_msgs
  shape_msgs
  rclcpp_action
  franka_msgs
)

foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

add_executable(environment src/environment.cpp)
ament_target_dependencies(environment rclcpp std_msgs std_srvs moveit_ros_planning_interface moveit_msgs finger_rig_msgs shape_msgs)

add_executable(motion_control src/motion_control.cpp)
ament_target_dependencies(motion_control rclcpp std_msgs std_srvs moveit_ros_planning_interface moveit_msgs finger_rig_msgs rclcpp_action franka_msgs)

add_executable(vision src/vision.cpp)
ament_target_dependencies(vision rclcpp std_msgs sensor_msgs cv_bridge OpenCV std_srvs)

install(TARGETS
  environment
  motion_control
  vision
  DESTINATION lib/${PROJECT_NAME}
)

# rosidl_generate_interfaces(${PROJECT_NAME}
#   "srv/GoToPose.srv"
# )

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
