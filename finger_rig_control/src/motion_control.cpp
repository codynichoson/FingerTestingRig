#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/empty.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <finger_rig_msgs/srv/go_to_pose.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <vector>

using std::placeholders::_1, std::placeholders::_2;

const std::string MOVE_GROUP = "panda_manipulator";

class MotionControl : public rclcpp::Node
{
  public:
    MotionControl();

    // Move group interface for the robot
    moveit::planning_interface::MoveGroupInterface move_group_;

  private:
    // Initialize services
    rclcpp::Service<std_srvs::srv::Empty>::SharedPtr home_srv_;
    rclcpp::Service<std_srvs::srv::Empty>::SharedPtr extend_srv_;
    rclcpp::Service<finger_rig_msgs::srv::GoToPose>::SharedPtr go_to_pose_srv_;
    rclcpp::Service<std_srvs::srv::Empty>::SharedPtr go_to_standoff_srv_;

    // Initalize service callback functions
    void home_srv_callback(const std::shared_ptr<std_srvs::srv::Empty::Request>, std::shared_ptr<std_srvs::srv::Empty::Response>);
    void extend_srv_callback(const std::shared_ptr<std_srvs::srv::Empty::Request>, std::shared_ptr<std_srvs::srv::Empty::Response>);
    void go_to_pose_srv_callback(const std::shared_ptr<finger_rig_msgs::srv::GoToPose::Request>, std::shared_ptr<finger_rig_msgs::srv::GoToPose::Response>);
    void go_to_standoff_srv_callback(const std::shared_ptr<std_srvs::srv::Empty::Request>, std::shared_ptr<std_srvs::srv::Empty::Response>);

    // Initalize functions
    void go_to_pose(double x, double y, double z, double roll, double pitch, double yaw);
};

MotionControl::MotionControl() : Node("motion_control"), move_group_(std::shared_ptr<rclcpp::Node>(std::move(this)), MOVE_GROUP)
{
  //// CONSTRUCTOR
  // Create services
  home_srv_ = create_service<std_srvs::srv::Empty>("home", std::bind(&MotionControl::home_srv_callback, this, _1, _2));
  extend_srv_ = create_service<std_srvs::srv::Empty>("extend", std::bind(&MotionControl::extend_srv_callback, this, _1, _2));
  go_to_pose_srv_ = create_service<finger_rig_msgs::srv::GoToPose>("go_to_pose", std::bind(&MotionControl::go_to_pose_srv_callback, this, _1, _2));
  go_to_standoff_srv_ = create_service<std_srvs::srv::Empty>("go_to_standoff", std::bind(&MotionControl::go_to_standoff_srv_callback, this, _1, _2));

  // Use upper joint velocity and acceleration limits
  this->move_group_.setMaxAccelerationScalingFactor(1.0);
  this->move_group_.setMaxVelocityScalingFactor(1.0);

  RCLCPP_INFO(this->get_logger(), "Initialization successful.");
}

void MotionControl::home_srv_callback(const std::shared_ptr<std_srvs::srv::Empty::Request>, std::shared_ptr<std_srvs::srv::Empty::Response>)
{
  RCLCPP_INFO(this->get_logger(), "Home service called.");

  // Set named target for home pose and execute
  this->move_group_.setNamedTarget("ready");
  this->move_group_.move();
}

void MotionControl::extend_srv_callback(const std::shared_ptr<std_srvs::srv::Empty::Request>, std::shared_ptr<std_srvs::srv::Empty::Response>)
{
  RCLCPP_INFO(this->get_logger(), "Extend service called.");

  // Set named target for extended pose and execute
  this->move_group_.setNamedTarget("extended");
  this->move_group_.move();
}

void MotionControl::go_to_pose_srv_callback(const std::shared_ptr<finger_rig_msgs::srv::GoToPose::Request> req, std::shared_ptr<finger_rig_msgs::srv::GoToPose::Response>)
{
  RCLCPP_INFO(this->get_logger(), "Go to pose service called.");

  this->go_to_pose(req->x, req->y, req->z, req->roll, req->pitch, req->yaw);
}

void MotionControl::go_to_standoff_srv_callback(const std::shared_ptr<std_srvs::srv::Empty::Request>, std::shared_ptr<std_srvs::srv::Empty::Response>)
{
  RCLCPP_INFO(this->get_logger(), "Standoff service called.");

  this->go_to_pose(0.3, 0.1, 0.35, 3.14, 0.0, 0.0);
}

void MotionControl::go_to_pose(double x, double y, double z, double roll, double pitch, double yaw)
{
  // Create variables for waypoint
  std::vector<geometry_msgs::msg::Pose> waypoints;
  geometry_msgs::msg::Pose target_pose;
  tf2::Quaternion myQuaternion;

  // Define waypoint
  myQuaternion.setRPY(roll, pitch, yaw);
  myQuaternion = myQuaternion.normalize();
  target_pose.position.x = x;
  target_pose.position.y = y;
  target_pose.position.z = z;
  target_pose.orientation.x = myQuaternion.x();
  target_pose.orientation.y = myQuaternion.y();
  target_pose.orientation.z = myQuaternion.z();
  target_pose.orientation.w = myQuaternion.w();
  waypoints.push_back(target_pose);

  // Compute Cartesian path
  moveit_msgs::msg::RobotTrajectory trajectory;
  const double jump_threshold = 0.0;
  const double eef_step = 0.01;
  double fraction = move_group_.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);
  RCLCPP_INFO(this->get_logger(), "Visualizing plan (Cartesian path) (%.2f%% achieved)", fraction * 100.0);

  // Execute Cartesian path
  move_group_.execute(trajectory);
  RCLCPP_INFO(this->get_logger(), "Cartesian path successfully executed");
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MotionControl>());
  rclcpp::shutdown();
  return 0;
}