#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/objdetect/objdetect.hpp>
#include <opencv2/highgui/highgui.hpp>

using std::placeholders::_1;

/// \brief Vision node class
class Vision : public rclcpp::Node
{
  public:
    // Constructor
    Vision() : Node("vision")
    {
      image_sub_ = this->create_subscription<sensor_msgs::msg::Image>("/camera/image", 10, std::bind(&Vision::image_callback, this, _1));
      cv::namedWindow("Image Window");
    }

    // Destructor
    ~Vision()
    {
      cv::destroyWindow("Image Window");
    }

  private:
    // void image_callback(const sensor_msgs::msg::Image::SharedPtr msg) const
    void image_callback(const sensor_msgs::msg::Image::SharedPtr msg) const
    {
      // Display image characteristics
      // RCLCPP_INFO(this->get_logger(),
      //   "Right Rectified image received from ZED\tSize: %dx%d - Timestamp: %u.%u sec ",
      //   msg->width, msg->height,
      //   msg->header.stamp.sec,msg->header.stamp.nanosec);

      // Convert ROS sensor_msgs/Image to OpenCV pointer type
      // CvImagePtr cv_bridge::toCvCopy(const sensor_msgs::ImageConstPtr& msg, const std::string& encoding = std::string());
      // CvImagePtr cv_bridge::toCvCopy(const sensor_msgs::msg::Image& msg, const std::string& encoding = std::string());
      // cv_bridge::CvImagePtr cv_img_ptr = NULL;
      cv_bridge::CvImagePtr cv_img_ptr;

      cv_img_ptr = cv_bridge::toCvCopy(msg, msg->encoding);

      cv::cvShowImage("Image Window", *cv_img_ptr);
      cv::waitKey(3);
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