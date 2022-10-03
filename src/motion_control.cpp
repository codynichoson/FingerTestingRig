#include "rclcpp/rclcpp.hpp"

class MotionControl : public rclcpp::Node
{
    public:
        MotionControl() : Node("motion_control") {}
    private:
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MotionControl>();
    rclcpp::spin(node);
    rclcpp::shutdown();
}
