#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/objdetect/objdetect.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <std_srvs/srv/empty.hpp>
#include <std_msgs/msg/header.h>
// #include <clock.hpp>

using std::placeholders::_1, std::placeholders::_2;

static const std::string OPENCV_WINDOW = "Image Window";

/// \brief Vision node class
class Vision : public rclcpp::Node
{
  public:
    // Constructor
    Vision() : Node("vision")
    {
      // Create image subscriber
      image_sub_ = this->create_subscription<sensor_msgs::msg::Image>("/camera/image", 10, std::bind(&Vision::image_sub_callback, this, _1));

      // Construct publishers
      norm_pub_ = this->create_publisher<sensor_msgs::msg::Image>("/camera/image_normalized", 10);
      mask_pub_ = this->create_publisher<sensor_msgs::msg::Image>("/camera/image_mask", 10);

      // Create services
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

    // Initialize publishers
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr norm_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr mask_pub_;

    // Initialize services
    rclcpp::Service<std_srvs::srv::Empty>::SharedPtr capture_img_srv_;

    // Initialize variables
    cv_bridge::CvImagePtr cv_img_ptr;
    cv::Mat color_img;
    cv::Mat gray_img;
    cv::Mat normalized_img;
    cv::Mat mask;

    void image_sub_callback(const sensor_msgs::msg::Image::SharedPtr msg)
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

      // Get CV image matrix by dereferencing image within cv_bridge pointer
      // color_img = cv_img_ptr->image;
      cv::cvtColor(cv_img_ptr->image, color_img, CV_BGR2RGB);

      normalize_img();
      threshold_img();
    }

    void capture_img_srv_callback(const std::shared_ptr<std_srvs::srv::Empty::Request>, std::shared_ptr<std_srvs::srv::Empty::Response>)
    {
      RCLCPP_INFO(this->get_logger(), "Capturing image.");

      // Save image in /images directory in working directory
      bool check = cv::imwrite("images/color.jpg", color_img);

      // find_circles();
        
      // Check if image was successfully saved
      if (check == false) {
        RCLCPP_ERROR(this->get_logger(), "Image failed to save.");
      }
      else{
        RCLCPP_ERROR(this->get_logger(), "Image successfully saved.");
      }
    }

    void normalize_img()
    {
      cv::cvtColor(color_img, gray_img, cv::COLOR_RGB2GRAY);

      double min, max;
      cv::minMaxLoc(gray_img,&min,&max);
      float sub = min;
      float mult = 255.0f/(float)(max-sub);
      normalized_img = gray_img - sub;
      normalized_img = mult * normalized_img;

      std_msgs::msg::Header header;
      header.stamp = this->get_clock()->now();
      header.frame_id = "camera";

      sensor_msgs::msg::Image norm_pub_msg;
      cv_bridge::CvImage norm_cv_img;
      norm_cv_img = cv_bridge::CvImage(header, sensor_msgs::image_encodings::MONO8, normalized_img);
      norm_cv_img.toImageMsg(norm_pub_msg);

      norm_pub_->publish(norm_pub_msg);
    }

    void threshold_img()
    {
      cv::Mat mask;
      cv::threshold(normalized_img, mask, 110, 255, CV_THRESH_BINARY);

      std_msgs::msg::Header header;
      header.stamp = this->get_clock()->now();
      header.frame_id = "camera";

      sensor_msgs::msg::Image mask_pub_msg;
      cv_bridge::CvImage mask_cv_img;
      mask_cv_img = cv_bridge::CvImage(header, sensor_msgs::image_encodings::MONO8, mask);
      mask_cv_img.toImageMsg(mask_pub_msg);

      mask_pub_->publish(mask_pub_msg);
    }

    void find_circles()
    {
      // normalize_img();

      // threshold_img();


      // cv::medianBlur(gray_img, gray_img, 5);

      // std::vector<cv::Vec3f> circles;

      // cv::HoughCircles(gray_img, circles, cv::HOUGH_GRADIENT, 2, 5, 100, 100, 0, 1000);

      // RCLCPP_INFO(this->get_logger(), "Circles detected: %d", circles.size());

      // for(std::size_t i = 0; i < circles.size(); i++)
      // {
      //   cv::Vec3i c = circles[i];
      //   cv::Point center = cv::Point(c[0], c[1]);
      //   // circle center
      //   circle(color_img, center, 1, cv::Scalar(0,100,100), 3, cv::LINE_AA);
      //   // circle outline
      //   int radius = c[2];
      //   circle(color_img, center, radius, cv::Scalar(255,0,255), 3, cv::LINE_AA);
      // }

      // Save image in /images directory in working directory
      // bool check = cv::imwrite("images/normalized_img.jpg", normalized_img);
      // cv::imwrite("images/mask.jpg", mask);

      // cv::imshow(OPENCV_WINDOW, cv_img_ptr->image);
      // cv::waitKey(3);
        
      // // Check if image was successfully saved
      // if (check == false) {
      //   RCLCPP_ERROR(this->get_logger(), "Image failed to save.");
      // }
      // else{
      //   RCLCPP_ERROR(this->get_logger(), "Image successfully saved.");
      // }
    }
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Vision>());
  rclcpp::shutdown();
  return 0;
}