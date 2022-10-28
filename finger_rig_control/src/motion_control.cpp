#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/empty.hpp>

#include <moveit/move_group_interface/move_group_interface.h>

using std::placeholders::_1, std::placeholders::_2;

const std::string MOVE_GROUP = "panda_arm";

class MotionControl : public rclcpp::Node
{
  public:
    MotionControl();

    /// Move group interface for the robot
    moveit::planning_interface::MoveGroupInterface move_group_;

  private:
    // Initialize publisher
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;

    // Initialize services
    rclcpp::Service<std_srvs::srv::Empty>::SharedPtr home_srv_;
    rclcpp::Service<std_srvs::srv::Empty>::SharedPtr extend_srv_;

    // Initalize callback functions
    void home_srv_callback(const std::shared_ptr<std_srvs::srv::Empty::Request>, std::shared_ptr<std_srvs::srv::Empty::Response>);
    void extend_srv_callback(const std::shared_ptr<std_srvs::srv::Empty::Request>, std::shared_ptr<std_srvs::srv::Empty::Response>);
};

MotionControl::MotionControl() : Node("motion_control"), move_group_(std::shared_ptr<rclcpp::Node>(std::move(this)), MOVE_GROUP)
{
  // Create publisher
  publisher_ = this->create_publisher<std_msgs::msg::String>("topic", 10);

  // Create services
  home_srv_ = create_service<std_srvs::srv::Empty>("home", std::bind(&MotionControl::home_srv_callback, this, _1, _2));
  extend_srv_ = create_service<std_srvs::srv::Empty>("extend", std::bind(&MotionControl::extend_srv_callback, this, _1, _2));

  RCLCPP_INFO(this->get_logger(), "BLAH BLAH BLAH.");

  // Use upper joint velocity and acceleration limits
  this->move_group_.setMaxAccelerationScalingFactor(1.0);
  this->move_group_.setMaxVelocityScalingFactor(1.0);

  RCLCPP_INFO(this->get_logger(), "Initialization successful.");
}

void MotionControl::home_srv_callback(const std::shared_ptr<std_srvs::srv::Empty::Request>, std::shared_ptr<std_srvs::srv::Empty::Response>)
{
  RCLCPP_INFO(this->get_logger(), "Home service called.");

  // Plan and execute motion
  // this->move_group_.setPoseTarget(msg->pose);
  this->move_group_.setNamedTarget("ready");
  this->move_group_.move();
}

void MotionControl::extend_srv_callback(const std::shared_ptr<std_srvs::srv::Empty::Request>, std::shared_ptr<std_srvs::srv::Empty::Response>)
{
  RCLCPP_INFO(this->get_logger(), "Extend service called.");
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MotionControl>());
  rclcpp::shutdown();
  return 0;
}