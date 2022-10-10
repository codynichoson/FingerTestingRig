#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"

using std::placeholders::_1;

class Vision : public rclcpp::Node
{
  public:
    // Constructor
    Vision() : Node("minimal_subscriber")
    {
      image_sub_ = this->create_subscription<sensor_msgs::msg::Image>("/camera/image", 10, std::bind(&Vision::topic_callback, this, _1));
    }

  private:
    void topic_callback(const sensor_msgs::msg::Image::SharedPtr msg) const
    {
      RCLCPP_INFO(this->get_logger(),
        "Right Rectified image received from ZED\tSize: %dx%d - Timestamp: %u.%u sec ",
        msg->width, msg->height,
        msg->header.stamp.sec,msg->header.stamp.nanosec);
    }
    
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Vision>());
  rclcpp::shutdown();
  return 0;
}