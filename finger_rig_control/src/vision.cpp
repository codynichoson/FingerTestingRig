#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/objdetect/objdetect.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <std_srvs/srv/empty.hpp>

using std::placeholders::_1, std::placeholders::_2;

/// \brief Vision node class
class Vision : public rclcpp::Node
{
  public:
    // Constructor
    Vision() : Node("vision")
    {
      // Create image subscriber
      image_sub_ = this->create_subscription<sensor_msgs::msg::Image>("/camera/image", 10, std::bind(&Vision::image_callback, this, _1));

      // Create services
      stream_srv_ = create_service<std_srvs::srv::Empty>("stream_camera", std::bind(&Vision::stream_srv_callback, this, _1, _2));
      capture_img_srv_ = create_service<std_srvs::srv::Empty>("capture_image", std::bind(&Vision::capture_img_srv_callback, this, _1, _2));

      cv::namedWindow("Image Window");      
    }

    // Destructor
    ~Vision()
    {
      cv::destroyWindow("Image Window");
    }

  private:
  // Initialize subscribers
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;

    // Initialize services
    rclcpp::Service<std_srvs::srv::Empty>::SharedPtr stream_srv_;
    rclcpp::Service<std_srvs::srv::Empty>::SharedPtr capture_img_srv_;

    // Initialize variables
    cv_bridge::CvImagePtr cv_img_ptr;
    cv::Mat img_mat;

    void image_callback(const sensor_msgs::msg::Image::SharedPtr msg)
    {
      try
      {
        cv_img_ptr = cv_bridge::toCvCopy(msg, msg->encoding);
      }
      catch(cv_bridge::Exception& e)
      {
        RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
        return;
      }

      // Convert cv_bridge pointer to cv image matrix
      img_mat = cv_img_ptr->image;
    }

    void stream_srv_callback(const std::shared_ptr<std_srvs::srv::Empty::Request>, std::shared_ptr<std_srvs::srv::Empty::Response>)
    {
      RCLCPP_INFO(this->get_logger(), "Streaming from camera.");

      while(true){
        cv::imshow("Image Window", cv_img_ptr->image);
        cv::waitKey(3);
      }
    }

    void capture_img_srv_callback(const std::shared_ptr<std_srvs::srv::Empty::Request>, std::shared_ptr<std_srvs::srv::Empty::Response>)
    {
      RCLCPP_INFO(this->get_logger(), "Capturing image.");

      // Save image in /images directory in working directory
      bool check = cv::imwrite("images/MyImage.jpg", img_mat);
        
      // Check if image was successfully saved
      if (check == false) {
        RCLCPP_ERROR(this->get_logger(), "Image failed to save.");
      }
      else{
        RCLCPP_ERROR(this->get_logger(), "Image successfully saved.");
      }
        
    }
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Vision>());
  rclcpp::shutdown();
  return 0;
}