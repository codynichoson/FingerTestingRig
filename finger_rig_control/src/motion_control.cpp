#include <rclcpp/rclcpp.hpp>
#include <rclcpp/utilities.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/empty.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <finger_rig_msgs/srv/go_to_pose.hpp>
#include <finger_rig_msgs/srv/set_gripper.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <rclcpp_action/rclcpp_action.hpp>
#include <franka_msgs/action/grasp.hpp>
#include <vector>
#include <chrono>

using std::placeholders::_1, std::placeholders::_2;

const std::string ARM_MOVE_GROUP = "panda_manipulator";
const std::string HAND_MOVE_GROUP = "hand";

class MotionControl : public rclcpp::Node
{
  public:
    MotionControl();

    // Move group interface for the robot
    moveit::planning_interface::MoveGroupInterface arm_move_group_;
    moveit::planning_interface::MoveGroupInterface hand_move_group_;

  private:
    // Initialize services
    rclcpp::Service<std_srvs::srv::Empty>::SharedPtr home_srv_;
    rclcpp::Service<std_srvs::srv::Empty>::SharedPtr extend_srv_;
    rclcpp::Service<std_srvs::srv::Empty>::SharedPtr open_srv_;
    rclcpp::Service<std_srvs::srv::Empty>::SharedPtr close_srv_;
    rclcpp::Service<finger_rig_msgs::srv::SetGripper>::SharedPtr set_gripper_srv_;
    rclcpp::Service<finger_rig_msgs::srv::GoToPose>::SharedPtr go_to_pose_srv_;
    rclcpp::Service<std_srvs::srv::Empty>::SharedPtr go_to_standoff_srv_;
    rclcpp::Service<std_srvs::srv::Empty>::SharedPtr rectangle_srv_;
    rclcpp::Service<std_srvs::srv::Empty>::SharedPtr linear_srv_;
    rclcpp::Service<std_srvs::srv::Empty>::SharedPtr circle_srv_;

    // Initialize action clients
    rclcpp_action::Client<franka_msgs::action::Grasp>::SharedPtr client_ptr_;

    // Initalize service callback functions
    void home_srv_callback(const std::shared_ptr<std_srvs::srv::Empty::Request>, std::shared_ptr<std_srvs::srv::Empty::Response>);
    void extend_srv_callback(const std::shared_ptr<std_srvs::srv::Empty::Request>, std::shared_ptr<std_srvs::srv::Empty::Response>);
    void open_srv_callback(const std::shared_ptr<std_srvs::srv::Empty::Request>, std::shared_ptr<std_srvs::srv::Empty::Response>);
    void close_srv_callback(const std::shared_ptr<std_srvs::srv::Empty::Request>, std::shared_ptr<std_srvs::srv::Empty::Response>);
    void set_gripper_srv_callback(const std::shared_ptr<finger_rig_msgs::srv::SetGripper::Request>, std::shared_ptr<finger_rig_msgs::srv::SetGripper::Response>);
    void go_to_pose_srv_callback(const std::shared_ptr<finger_rig_msgs::srv::GoToPose::Request>, std::shared_ptr<finger_rig_msgs::srv::GoToPose::Response>);
    void go_to_standoff_srv_callback(const std::shared_ptr<std_srvs::srv::Empty::Request>, std::shared_ptr<std_srvs::srv::Empty::Response>);
    void rectangle_srv_callback(const std::shared_ptr<std_srvs::srv::Empty::Request>, std::shared_ptr<std_srvs::srv::Empty::Response>);
    void linear_srv_callback(const std::shared_ptr<std_srvs::srv::Empty::Request>, std::shared_ptr<std_srvs::srv::Empty::Response>);
    void circle_srv_callback(const std::shared_ptr<std_srvs::srv::Empty::Request>, std::shared_ptr<std_srvs::srv::Empty::Response>);

    // Initalize functions
    void go_to_pose(double x, double y, double z, double roll, double pitch, double yaw);
    void add_waypoint(double x, double y, double z, double roll, double pitch, double yaw);
    void grasp(double width, double speed, double force);
    void move_gripper(std::string pose_name);
    void move_gripper(double x, double y);

    // Initialize structs
    struct {
      double x;
      double y;
      double z;
      double roll;
      double pitch;
      double yaw;
    } standoff_pose;

    std::vector<geometry_msgs::msg::Pose> the_waypoints;
};

MotionControl::MotionControl() : Node("motion_control"), arm_move_group_(std::shared_ptr<rclcpp::Node>(std::move(this)), ARM_MOVE_GROUP), hand_move_group_(std::shared_ptr<rclcpp::Node>(std::move(this)), HAND_MOVE_GROUP)
{
  //// CONSTRUCTOR
  // Create services
  home_srv_ = create_service<std_srvs::srv::Empty>("home", std::bind(&MotionControl::home_srv_callback, this, _1, _2));
  extend_srv_ = create_service<std_srvs::srv::Empty>("extend", std::bind(&MotionControl::extend_srv_callback, this, _1, _2));
  open_srv_ = create_service<std_srvs::srv::Empty>("open_gripper", std::bind(&MotionControl::open_srv_callback, this, _1, _2));
  close_srv_ = create_service<std_srvs::srv::Empty>("close_gripper", std::bind(&MotionControl::close_srv_callback, this, _1, _2));
  set_gripper_srv_ = create_service<finger_rig_msgs::srv::SetGripper>("set_gripper", std::bind(&MotionControl::set_gripper_srv_callback, this, _1, _2));
  go_to_pose_srv_ = create_service<finger_rig_msgs::srv::GoToPose>("go_to_pose", std::bind(&MotionControl::go_to_pose_srv_callback, this, _1, _2));
  go_to_standoff_srv_ = create_service<std_srvs::srv::Empty>("go_to_standoff", std::bind(&MotionControl::go_to_standoff_srv_callback, this, _1, _2));
  rectangle_srv_ = create_service<std_srvs::srv::Empty>("rectangle", std::bind(&MotionControl::rectangle_srv_callback, this, _1, _2));
  linear_srv_ = create_service<std_srvs::srv::Empty>("linear", std::bind(&MotionControl::linear_srv_callback, this, _1, _2));
  circle_srv_ = create_service<std_srvs::srv::Empty>("circle", std::bind(&MotionControl::circle_srv_callback, this, _1, _2));

  // Create actions
  client_ptr_ = rclcpp_action::create_client<franka_msgs::action::Grasp>(this, "/panda_gripper/grasp");

  // Use upper joint velocity and acceleration limits
  // this->arm_move_group_.setMaxAccelerationScalingFactor(0.5);
  // this->arm_move_group_.setMaxVelocityScalingFactor(1.0);

  // this->hand_move_group_.setMaxAccelerationScalingFactor(1.0);
  // this->hand_move_group_.setMaxVelocityScalingFactor(1.0);

  standoff_pose.x = 0.74;
  standoff_pose.y = 0.12;
  standoff_pose.z = 0.065;
  standoff_pose.roll = 3.14;
  standoff_pose.pitch = 0.0;
  standoff_pose.yaw = 1.57;

  RCLCPP_INFO(this->get_logger(), "Initialization successful.");
}

void MotionControl::home_srv_callback(const std::shared_ptr<std_srvs::srv::Empty::Request>, std::shared_ptr<std_srvs::srv::Empty::Response>)
{
  RCLCPP_INFO(this->get_logger(), "Home service called.");

  // Set named target for home pose and execute
  this->arm_move_group_.setNamedTarget("ready");
  this->arm_move_group_.move();
}

void MotionControl::extend_srv_callback(const std::shared_ptr<std_srvs::srv::Empty::Request>, std::shared_ptr<std_srvs::srv::Empty::Response>)
{
  RCLCPP_INFO(this->get_logger(), "Extend service called.");

  // Set named target for extended pose and execute
  this->arm_move_group_.setNamedTarget("extended");
  this->arm_move_group_.move();
}

void MotionControl::open_srv_callback(const std::shared_ptr<std_srvs::srv::Empty::Request>, std::shared_ptr<std_srvs::srv::Empty::Response>)
{
  RCLCPP_INFO(this->get_logger(), "Open gripper service called.");
  this->move_gripper("open");
}

void MotionControl::close_srv_callback(const std::shared_ptr<std_srvs::srv::Empty::Request>, std::shared_ptr<std_srvs::srv::Empty::Response>)
{
  RCLCPP_INFO(this->get_logger(), "Close gripper service called.");
  // this->move_gripper("close");
  this->grasp(0.036, 0.03, 10);
}

void MotionControl::set_gripper_srv_callback(const std::shared_ptr<finger_rig_msgs::srv::SetGripper::Request> req, std::shared_ptr<finger_rig_msgs::srv::SetGripper::Response>)
{
  RCLCPP_INFO(this->get_logger(), "Set gripper service called.");

  this->move_gripper(req->left, req->right);
}

void MotionControl::go_to_pose_srv_callback(const std::shared_ptr<finger_rig_msgs::srv::GoToPose::Request> req, std::shared_ptr<finger_rig_msgs::srv::GoToPose::Response>)
{
  RCLCPP_INFO(this->get_logger(), "Go to pose service called.");

  this->go_to_pose(req->x, req->y, req->z, req->roll, req->pitch, req->yaw);
}

void MotionControl::go_to_standoff_srv_callback(const std::shared_ptr<std_srvs::srv::Empty::Request>, std::shared_ptr<std_srvs::srv::Empty::Response>)
{
  RCLCPP_INFO(this->get_logger(), "Standoff service called.");

  this->go_to_pose(standoff_pose.x, standoff_pose.y, 0.25, standoff_pose.roll, standoff_pose.pitch, standoff_pose.yaw);
  this->go_to_pose(standoff_pose.x, standoff_pose.y, standoff_pose.z, standoff_pose.roll, standoff_pose.pitch, standoff_pose.yaw);
}

void MotionControl::rectangle_srv_callback(const std::shared_ptr<std_srvs::srv::Empty::Request>, std::shared_ptr<std_srvs::srv::Empty::Response>)
{
  RCLCPP_INFO(this->get_logger(), "Rectangle service called.");

  this->go_to_pose(standoff_pose.x, standoff_pose.y, 0.25, standoff_pose.roll, standoff_pose.pitch, standoff_pose.yaw);
  this->go_to_pose(standoff_pose.x, standoff_pose.y, standoff_pose.z, standoff_pose.roll, standoff_pose.pitch, standoff_pose.yaw);

  this->grasp(0.036, 0.03, 10); // close

  double rect_size = 0.03;

  // this->go_to_pose(standoff_pose.x + rect_size/2.0, standoff_pose.y,             standoff_pose.z, standoff_pose.roll, standoff_pose.pitch, standoff_pose.yaw);
  // this->go_to_pose(standoff_pose.x + rect_size/2.0, standoff_pose.y - rect_size, standoff_pose.z, standoff_pose.roll, standoff_pose.pitch, standoff_pose.yaw);
  // this->go_to_pose(standoff_pose.x - rect_size/2.0, standoff_pose.y - rect_size, standoff_pose.z, standoff_pose.roll, standoff_pose.pitch, standoff_pose.yaw);
  // this->go_to_pose(standoff_pose.x - rect_size/2.0, standoff_pose.y,             standoff_pose.z, standoff_pose.roll, standoff_pose.pitch, standoff_pose.yaw);
  // this->go_to_pose(standoff_pose.x,                 standoff_pose.y,             standoff_pose.z, standoff_pose.roll, standoff_pose.pitch, standoff_pose.yaw);

  this->add_waypoint(standoff_pose.x + rect_size/2.0, standoff_pose.y,             standoff_pose.z, standoff_pose.roll, standoff_pose.pitch, standoff_pose.yaw);
  this->add_waypoint(standoff_pose.x + rect_size/2.0, standoff_pose.y - rect_size, standoff_pose.z, standoff_pose.roll, standoff_pose.pitch, standoff_pose.yaw);
  this->add_waypoint(standoff_pose.x - rect_size/2.0, standoff_pose.y - rect_size, standoff_pose.z, standoff_pose.roll, standoff_pose.pitch, standoff_pose.yaw);
  this->add_waypoint(standoff_pose.x - rect_size/2.0, standoff_pose.y,             standoff_pose.z, standoff_pose.roll, standoff_pose.pitch, standoff_pose.yaw);
  this->add_waypoint(standoff_pose.x,                 standoff_pose.y,             standoff_pose.z, standoff_pose.roll, standoff_pose.pitch, standoff_pose.yaw);

  // Compute Cartesian path
  moveit_msgs::msg::RobotTrajectory trajectory;
  const double jump_threshold = 0.0;
  const double eef_step = 0.01;
  double fraction = arm_move_group_.computeCartesianPath(the_waypoints, eef_step, jump_threshold, trajectory);
  RCLCPP_INFO(this->get_logger(), "Visualizing plan (Cartesian path) (%.2f%% achieved)", fraction * 100.0);

  // Execute Cartesian path
  arm_move_group_.execute(trajectory);
  RCLCPP_INFO(this->get_logger(), "Cartesian path successfully executed");

  the_waypoints.clear();

  this->grasp(0.08, 0.03, 10); // open
}

void MotionControl::linear_srv_callback(const std::shared_ptr<std_srvs::srv::Empty::Request>, std::shared_ptr<std_srvs::srv::Empty::Response>)
{
  RCLCPP_INFO(this->get_logger(), "Linear service called.");

  this->go_to_pose(standoff_pose.x, standoff_pose.y, 0.25, standoff_pose.roll, standoff_pose.pitch, standoff_pose.yaw);
  this->go_to_pose(standoff_pose.x, standoff_pose.y, standoff_pose.z, standoff_pose.roll, standoff_pose.pitch, standoff_pose.yaw);

  double dist = 0.04;

  this->grasp(0.036, 0.03, 10); // close
  
  this->go_to_pose(standoff_pose.x, standoff_pose.y - dist, standoff_pose.z, standoff_pose.roll, standoff_pose.pitch, standoff_pose.yaw);
  this->go_to_pose(standoff_pose.x, standoff_pose.y, standoff_pose.z, standoff_pose.roll, standoff_pose.pitch, standoff_pose.yaw);
  this->go_to_pose(standoff_pose.x, standoff_pose.y - dist, standoff_pose.z, standoff_pose.roll, standoff_pose.pitch, standoff_pose.yaw);
  this->go_to_pose(standoff_pose.x, standoff_pose.y, standoff_pose.z, standoff_pose.roll, standoff_pose.pitch, standoff_pose.yaw);
  this->go_to_pose(standoff_pose.x, standoff_pose.y - dist, standoff_pose.z, standoff_pose.roll, standoff_pose.pitch, standoff_pose.yaw);
  this->go_to_pose(standoff_pose.x, standoff_pose.y, standoff_pose.z, standoff_pose.roll, standoff_pose.pitch, standoff_pose.yaw);

  this->grasp(0.08, 0.03, 10); // open

  // this->add_waypoint(standoff_pose.x, standoff_pose.y - dist, standoff_pose.z, standoff_pose.roll, standoff_pose.pitch, standoff_pose.yaw);
  // this->add_waypoint(standoff_pose.x, standoff_pose.y, standoff_pose.z, standoff_pose.roll, standoff_pose.pitch, standoff_pose.yaw);
  // this->add_waypoint(standoff_pose.x, standoff_pose.y - dist, standoff_pose.z, standoff_pose.roll, standoff_pose.pitch, standoff_pose.yaw);
  // this->add_waypoint(standoff_pose.x, standoff_pose.y, standoff_pose.z, standoff_pose.roll, standoff_pose.pitch, standoff_pose.yaw);
  // this->add_waypoint(standoff_pose.x, standoff_pose.y - dist, standoff_pose.z, standoff_pose.roll, standoff_pose.pitch, standoff_pose.yaw);
  // this->add_waypoint(standoff_pose.x, standoff_pose.y, standoff_pose.z, standoff_pose.roll, standoff_pose.pitch, standoff_pose.yaw);

  // Compute Cartesian path
  // moveit_msgs::msg::RobotTrajectory trajectory;
  // const double jump_threshold = 0.0;
  // const double eef_step = 0.01;
  // double fraction = arm_move_group_.computeCartesianPath(the_waypoints, eef_step, jump_threshold, trajectory);
  // RCLCPP_INFO(this->get_logger(), "Visualizing plan (Cartesian path) (%.2f%% achieved)", fraction * 100.0);

  // // Execute Cartesian path
  // arm_move_group_.execute(trajectory);
  // RCLCPP_INFO(this->get_logger(), "Cartesian path successfully executed");

  // the_waypoints.clear();
}

void MotionControl::circle_srv_callback(const std::shared_ptr<std_srvs::srv::Empty::Request>, std::shared_ptr<std_srvs::srv::Empty::Response>)
{
  RCLCPP_INFO(this->get_logger(), "Circle service called.");

  this->go_to_pose(standoff_pose.x, standoff_pose.y, 0.25, standoff_pose.roll, standoff_pose.pitch, standoff_pose.yaw);
  this->go_to_pose(standoff_pose.x, standoff_pose.y, standoff_pose.z, standoff_pose.roll, standoff_pose.pitch, standoff_pose.yaw);

  this->grasp(0.036, 0.03, 10); // close

  RCLCPP_INFO(this->get_logger(), "Circle coordinates:");

  this->go_to_pose(standoff_pose.x, standoff_pose.y-0.02, standoff_pose.z, standoff_pose.roll, standoff_pose.pitch, standoff_pose.yaw);
  RCLCPP_INFO(this->get_logger(), "x: %f     y: %f    z: %f", standoff_pose.x, standoff_pose.y-0.02, standoff_pose.z);

  double radius = 0.02;

  // Create variables for waypoint
  std::vector<geometry_msgs::msg::Pose> waypoints;
  geometry_msgs::msg::Pose target_pose;
  tf2::Quaternion myQuaternion;

  for (double theta = 0.0; theta < 20.0; theta = theta + 1.0)
  {
    // Define waypoint
    myQuaternion.setRPY(standoff_pose.roll, standoff_pose.pitch, standoff_pose.yaw);
    myQuaternion = myQuaternion.normalize();
    target_pose.position.x = standoff_pose.x + radius*cos(theta*(180.0/3.14));
    target_pose.position.y = standoff_pose.y + radius*sin(theta*(180.0/3.14)) - 0.02;
    target_pose.position.z = standoff_pose.z;
    target_pose.orientation.x = myQuaternion.x();
    target_pose.orientation.y = myQuaternion.y();
    target_pose.orientation.z = myQuaternion.z();
    target_pose.orientation.w = myQuaternion.w();
    waypoints.push_back(target_pose);

    RCLCPP_INFO(this->get_logger(), "x: %f     y: %f    z: %f", target_pose.position.x, target_pose.position.y, target_pose.position.z);
  }

  myQuaternion.setRPY(standoff_pose.roll, standoff_pose.pitch, standoff_pose.yaw);
  myQuaternion = myQuaternion.normalize();
  target_pose.position.x = standoff_pose.x;
  target_pose.position.y = standoff_pose.y;
  target_pose.position.z = standoff_pose.z;
  target_pose.orientation.x = myQuaternion.x();
  target_pose.orientation.y = myQuaternion.y();
  target_pose.orientation.z = myQuaternion.z();
  target_pose.orientation.w = myQuaternion.w();
  waypoints.push_back(target_pose);

  RCLCPP_INFO(this->get_logger(), "x: %f     y: %f    z: %f", target_pose.position.x, target_pose.position.y, target_pose.position.z);

  // Compute Cartesian path
  moveit_msgs::msg::RobotTrajectory trajectory;
  const double jump_threshold = 0.0;
  const double eef_step = 0.01;
  double fraction = arm_move_group_.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);
  RCLCPP_INFO(this->get_logger(), "Visualizing plan (Cartesian path) (%.2f%% achieved)", fraction * 100.0);

  // Execute Cartesian path
  arm_move_group_.execute(trajectory);
  RCLCPP_INFO(this->get_logger(), "Cartesian path successfully executed");

  // this->move_gripper("open");
  this->grasp(0.08, 0.03, 10); // open
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
  double fraction = arm_move_group_.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);
  RCLCPP_INFO(this->get_logger(), "Visualizing plan (Cartesian path) (%.2f%% achieved)", fraction * 100.0);

  // Execute Cartesian path
  arm_move_group_.execute(trajectory);
  RCLCPP_INFO(this->get_logger(), "Cartesian path successfully executed");
}

void MotionControl::add_waypoint(double x, double y, double z, double roll, double pitch, double yaw)
{
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
  the_waypoints.push_back(target_pose);
}

void MotionControl::grasp(double width, double speed, double force)
{
  // if (!this->client_ptr_->wait_for_action_server()) {
  //   RCLCPP_ERROR(this->get_logger(), "Action server not available after waiting");
  //   rclcpp::shutdown();
  // }

  auto goal_msg = franka_msgs::action::Grasp::Goal();
  goal_msg.width = width;
  goal_msg.speed = speed;
  goal_msg.force = force;

  RCLCPP_INFO(this->get_logger(), "Sending goal");

  auto send_goal_options = rclcpp_action::Client<franka_msgs::action::Grasp>::SendGoalOptions();
  // send_goal_options.goal_response_callback = std::bind(&MotionControl::goal_response_callback, this, _1);
  // send_goal_options.feedback_callback = std::bind(&MotionControl::feedback_callback, this, _1, _2);
  // send_goal_options.result_callback = std::bind(&MotionControl::result_callback, this, _1);
  client_ptr_->async_send_goal(goal_msg, send_goal_options);

  std::chrono::milliseconds milli(3500);
  rclcpp::sleep_for(milli);

}

void MotionControl::move_gripper(std::string pose_name)
{
  this->hand_move_group_.setNamedTarget(pose_name);
  this->hand_move_group_.move();
}

void MotionControl::move_gripper(double x, double y)
{
  std::vector<double> joints = {x, y};

  this->hand_move_group_.setJointValueTarget(joints);
  this->hand_move_group_.move();
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MotionControl>());
  rclcpp::shutdown();
  return 0;
}