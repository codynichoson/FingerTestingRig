#include "rclcpp/rclcpp.hpp"

class Vision : public rclcpp::Node
{
    public:
        Vision() : Node("vision") {}
    private:
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<Vision>();
    rclcpp::spin(node);
    rclcpp::shutdown();
}