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
    // Initalize planning scene interface
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

    void place_objects();
    moveit_msgs::msg::CollisionObject create_collision_object(std::string id, double shape_x, double shape_y, double shape_z, double x, double y, double z);
    
};

Environment::Environment() : Node("environment"), move_group_(std::shared_ptr<rclcpp::Node>(std::move(this)), MOVE_GROUP), planning_scene_interface()
{
  //// CONSTRUCTOR
  RCLCPP_INFO(this->get_logger(), "Environment node initialization successful.");

  this->place_objects();
}

// void Environment::place_table()
// {
//     moveit_msgs::msg::CollisionObject table_collision_obj;
//     table_collision_obj.header.frame_id = move_group_.getPlanningFrame();
//     table_collision_obj.id = "table";
//     shape_msgs::msg::SolidPrimitive primitive;

//     // Define the size of the box in meters
//     primitive.type = primitive.BOX;
//     primitive.dimensions.resize(3);
//     primitive.dimensions[primitive.BOX_X] = 0.36;
//     primitive.dimensions[primitive.BOX_Y] = 0.36;
//     primitive.dimensions[primitive.BOX_Z] = 0.20;

//     // Define the pose of the box (relative to the frame_id)
//     geometry_msgs::msg::Pose box_pose;
//     box_pose.orientation.w = 1.0;
//     box_pose.position.x = 0.5;
//     box_pose.position.y = 0.0;
//     box_pose.position.z = primitive.dimensions[primitive.BOX_Z]/2;

//     table_collision_obj.primitives.push_back(primitive);
//     table_collision_obj.primitive_poses.push_back(box_pose);
//     table_collision_obj.operation = table_collision_obj.ADD;

//     std::vector<moveit_msgs::msg::CollisionObject> objects;
//     objects.push_back(table_collision_obj);

//     planning_scene_interface.applyCollisionObjects(objects);
// }

// void Environment::place_finger_depressor()
// {
//     moveit_msgs::msg::CollisionObject collision_object;
//     collision_object.header.frame_id = move_group_.getPlanningFrame();
//     collision_object.id = "finger_depressor";
//     shape_msgs::msg::SolidPrimitive primitive;

//     // Define the size of the box in meters
//     primitive.type = primitive.BOX;
//     primitive.dimensions.resize(3);
//     primitive.dimensions[primitive.BOX_X] = 0.36;
//     primitive.dimensions[primitive.BOX_Y] = 0.36;
//     primitive.dimensions[primitive.BOX_Z] = 0.20;

//     // Define the pose of the box (relative to the frame_id)
//     geometry_msgs::msg::Pose box_pose;
//     box_pose.orientation.w = 1.0;
//     box_pose.position.x = 0.5;
//     box_pose.position.y = 0.0;
//     box_pose.position.z = primitive.dimensions[primitive.BOX_Z]/2;

//     collision_object.primitives.push_back(primitive);
//     collision_object.primitive_poses.push_back(box_pose);
//     collision_object.operation = collision_object.ADD;

//     std::vector<moveit_msgs::msg::CollisionObject> objects;
//     objects.push_back(collision_object);

//     planning_scene_interface.applyCollisionObjects(objects);
// }

void Environment::place_objects()
{
    moveit_msgs::msg::CollisionObject table = this->create_collision_object("table", 0.36, 0.36, 0.20, 0.35, 0.0, 0.20/2);
    moveit_msgs::msg::CollisionObject finger_depressor = this->create_collision_object("finger_depressor", 0.36, 0.03, 0.09, 0.35, 0.0, 0.245);

    std::vector<moveit_msgs::msg::CollisionObject> objects;
    objects.push_back(table);
    objects.push_back(finger_depressor);

    planning_scene_interface.applyCollisionObjects(objects);
}

moveit_msgs::msg::CollisionObject Environment::create_collision_object(
    std::string id, double shape_x, double shape_y, double shape_z, double x, double y, double z)
{
    moveit_msgs::msg::CollisionObject collision_object;
    collision_object.header.frame_id = move_group_.getPlanningFrame();
    collision_object.id = id;
    shape_msgs::msg::SolidPrimitive primitive;

    // Define the size of the box in meters
    primitive.type = primitive.BOX;
    primitive.dimensions.resize(3);
    primitive.dimensions[primitive.BOX_X] = shape_x;
    primitive.dimensions[primitive.BOX_Y] = shape_y;
    primitive.dimensions[primitive.BOX_Z] = shape_z;

    // Define the pose of the box (relative to the frame_id)
    geometry_msgs::msg::Pose box_pose;
    box_pose.orientation.w = 1.0;
    box_pose.position.x = x;
    box_pose.position.y = y;
    box_pose.position.z = z;

    collision_object.primitives.push_back(primitive);
    collision_object.primitive_poses.push_back(box_pose);
    collision_object.operation = collision_object.ADD;

    return collision_object;
}

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);

    // Environment::place_table();

    rclcpp::spin(std::make_shared<Environment>());
    rclcpp::shutdown();
    return 0;
}