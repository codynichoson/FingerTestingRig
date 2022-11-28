#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>

#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/objdetect/objdetect.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgcodecs/imgcodecs.hpp>

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
      gray_pub_ = this->create_publisher<sensor_msgs::msg::Image>("/camera/image_gray", 10);
      norm_pub_ = this->create_publisher<sensor_msgs::msg::Image>("/camera/image_normalized", 10);
      mask_pub_ = this->create_publisher<sensor_msgs::msg::Image>("/camera/image_mask", 10);
      blur_pub_ = this->create_publisher<sensor_msgs::msg::Image>("/camera/image_blur", 10);
      contour_pub_ = this->create_publisher<sensor_msgs::msg::Image>("/camera/image_contours", 10);

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
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr gray_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr norm_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr mask_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr blur_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr contour_pub_;

    // Initialize variables
    cv_bridge::CvImagePtr cv_img_ptr;
    cv::Mat color_img;
    cv::Mat gray_img;
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
      color_img = cv_img_ptr->image;
      cv::cvtColor(color_img, gray_img, cv::COLOR_RGB2GRAY);

      std_msgs::msg::Header gray_header;
      gray_header.stamp = this->get_clock()->now();
      gray_header.frame_id = "camera";

      sensor_msgs::msg::Image gray_pub_msg;
      cv_bridge::CvImage gray_cv_img;
      gray_cv_img = cv_bridge::CvImage(gray_header, sensor_msgs::image_encodings::MONO8, gray_img);
      gray_cv_img.toImageMsg(gray_pub_msg);

      gray_pub_->publish(gray_pub_msg);

      threshold_img();
      find_contours();
    }

    void threshold_img()
    {
      cv::threshold(gray_img, mask, 160, 255, CV_THRESH_BINARY);

      // Invert threshold mask
      cv::bitwise_not(mask, mask);

      std_msgs::msg::Header header;
      header.stamp = this->get_clock()->now();
      header.frame_id = "camera";

      sensor_msgs::msg::Image mask_pub_msg;
      cv_bridge::CvImage mask_cv_img;
      mask_cv_img = cv_bridge::CvImage(header, sensor_msgs::image_encodings::MONO8, mask);
      mask_cv_img.toImageMsg(mask_pub_msg);

      mask_pub_->publish(mask_pub_msg);
    }

    void find_contours()
    {
      // Find contours in mask
      std::vector<std::vector<cv::Point>> contours;
      std::vector<cv::Vec4i> hierarchy;
      cv::Mat contour_output = mask.clone();
      cv::findContours(contour_output, contours, hierarchy, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_NONE);

      // Draw contours on color image
      cv::Mat drawing = color_img.clone();
      // for(int idx = 0; idx >= 0; idx = hierarchy[idx][0] )
      // {
      //   cv::Scalar color( rand()&255, rand()&255, rand()&255);
      //   cv::drawContours(drawing, contours, -1, color, cv::LINE_AA, 8, hierarchy);
      // }
      // cv::Scalar color(rand()&255, rand()&255, rand()&255);

      cv::Scalar red(255, 0, 0);
      cv::Scalar orange(255, 128, 0);
      cv::Scalar yellow(255, 255, 0);
      cv::Scalar green(0, 255, 0);
      cv::Scalar dark_green(0, 128, 0);
      cv::Scalar blue(0, 0, 255);
      cv::Scalar dark_blue(0, 0, 128);
      cv::Scalar indigo(128, 0, 255);
      cv::Scalar violet(255, 0, 255);
      cv::Scalar white(255, 255, 255);

      for (long unsigned int i = 0; i < contours.size(); i++)
      {
        if (i == 0)
        {
          cv::drawContours(drawing, contours, i, red, cv::LINE_AA, 8, hierarchy);
        }
        else if (i == 1)
        {
          cv::drawContours(drawing, contours, i, orange, cv::LINE_AA, 8, hierarchy);
        }
        else if (i == 2)
        {
          cv::drawContours(drawing, contours, i, green, cv::LINE_AA, 8, hierarchy);
        }
        else if (i == 3)
        {
          cv::drawContours(drawing, contours, i, dark_green, cv::LINE_AA, 8, hierarchy);
        }
        else if (i == 4)
        {
          cv::drawContours(drawing, contours, i, blue, cv::LINE_AA, 8, hierarchy);
        }
        else if (i == 5)
        {
          cv::drawContours(drawing, contours, i, dark_blue, cv::LINE_AA, 8, hierarchy);
        }
        else if (i == 6)
        {
          cv::drawContours(drawing, contours, i, indigo, cv::LINE_AA, 8, hierarchy);
        }
        else if (i == 7)
        {
          cv::drawContours(drawing, contours, i, violet, cv::LINE_AA, 8, hierarchy);
        }
        else
        {
          cv::drawContours(drawing, contours, i, white, cv::LINE_AA, 8, hierarchy);
        }
      }

      RCLCPP_INFO(this->get_logger(), "Number of contours: %ld", contours.size());

      // Publish image with contours drawn
      std_msgs::msg::Header contour_header;
      contour_header.stamp = this->get_clock()->now();
      contour_header.frame_id = "camera";
      sensor_msgs::msg::Image contour_pub_msg;
      cv_bridge::CvImage contour_cv_img;
      contour_cv_img = cv_bridge::CvImage(contour_header, sensor_msgs::image_encodings::RGB8, drawing);
      contour_cv_img.toImageMsg(contour_pub_msg);
      contour_pub_->publish(contour_pub_msg);
    }
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Vision>());
  rclcpp::shutdown();
  return 0;
}