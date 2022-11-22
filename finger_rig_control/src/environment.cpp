#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/empty.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <shape_msgs/msg/solid_primitive.hpp>
#include <vector>

using std::placeholders::_1, std::placeholders::_2;

const std::string MOVE_GROUP = "panda_manipulator";

class Environment : public rclcpp::Node
{
  public:
    Environment();

    // Move group interface for the robot
    moveit::planning_interface::MoveGroupInterface move_group_;

  private:
    // Initialize services
    rclcpp::Service<std_srvs::srv::Empty>::SharedPtr place_box_srv_;

    // Initalize service callback functions
    void place_box_srv_callback(const std::shared_ptr<std_srvs::srv::Empty::Request>, std::shared_ptr<std_srvs::srv::Empty::Response>);

    // Initalize functions
    void function();

    // Initalize planning scene interface
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
};

Environment::Environment() : Node("environment"), move_group_(std::shared_ptr<rclcpp::Node>(std::move(this)), MOVE_GROUP), planning_scene_interface()
{
  //// CONSTRUCTOR
  // Create services
  place_box_srv_ = create_service<std_srvs::srv::Empty>("place_box", std::bind(&Environment::place_box_srv_callback, this, _1, _2));

  RCLCPP_INFO(this->get_logger(), "Environment node initialization successful.");
}

void Environment::place_box_srv_callback(const std::shared_ptr<std_srvs::srv::Empty::Request>, std::shared_ptr<std_srvs::srv::Empty::Response>)
{
    RCLCPP_INFO(this->get_logger(), "Place box service called.");

    moveit_msgs::msg::CollisionObject collision_object;
    collision_object.header.frame_id = move_group_.getPlanningFrame();
    collision_object.id = "box1";
    shape_msgs::msg::SolidPrimitive primitive;

    // Define the size of the box in meters
    primitive.type = primitive.BOX;
    primitive.dimensions.resize(3);
    primitive.dimensions[primitive.BOX_X] = 0.5;
    primitive.dimensions[primitive.BOX_Y] = 0.1;
    primitive.dimensions[primitive.BOX_Z] = 0.5;

    // Define the pose of the box (relative to the frame_id)
    geometry_msgs::msg::Pose box_pose;
    box_pose.orientation.w = 1.0;
    box_pose.position.x = 0.2;
    box_pose.position.y = 0.2;
    box_pose.position.z = 0.25;

    collision_object.primitives.push_back(primitive);
    collision_object.primitive_poses.push_back(box_pose);
    collision_object.operation = collision_object.ADD;

    std::vector<moveit_msgs::msg::CollisionObject> objects;
    objects.push_back(collision_object);

    planning_scene_interface.applyCollisionObjects(objects);
}

void Environment::function()
{
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Environment>());
  rclcpp::shutdown();
  return 0;
}