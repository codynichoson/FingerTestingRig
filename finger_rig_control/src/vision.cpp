#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>

#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/objdetect/objdetect.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgcodecs/imgcodecs.hpp>
#include <opencv2/tracking.hpp>
#include <opencv2/tracking/tracking_legacy.hpp>
#include <opencv2/videoio/videoio.hpp>
#include <opencv2/core/utility.hpp>

#include <std_srvs/srv/empty.hpp>
#include <std_msgs/msg/header.h>

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
      // tracking_pub_ = this->create_publisher<sensor_msgs::msg::Image>("/camera/image_tracking", 10);

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
    // rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr tracking_pub_;

    // Initialize variables
    cv_bridge::CvImagePtr cv_img_ptr;
    cv::Mat color_img;
    cv::Mat gray_img;
    cv::Mat mask;

    bool initialize = true;

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
      // track_contours();
    }

    void threshold_img()
    {
      cv::threshold(gray_img, mask, 170, 255, CV_THRESH_BINARY);

      // Invert threshold mask
      cv::bitwise_not(mask, mask);

      // cv::Mat circle_mask = cv::Mat::zeros(mask.rows, mask.cols, CV_32F);
      cv::Mat circle_mask = cv::Mat::zeros(30, 30, CV_32F);
      // cv::Point center;
      // center.x = mask.rows/2.0;
      // center.y = mask.cols/2.0;
      // center.x = 1000;
      // center.y = 1000;
      // cv::circle(circle_mask, center, 500, (255, 255, 255), -1);

      // Combine mask and circle_mask and save to mask
      // cv::bitwise_and(mask, circle_mask, mask);

      // Publish mask image
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

      // Filter out any small contours
      std::vector<std::vector<cv::Point>> big_contours;
      for (long unsigned int i = 0; i < contours.size(); i++)
      {
        if (cv::contourArea(contours[i]) > 8)
        {
          big_contours.push_back(contours[i]);
        }
      }

      // Find centroids
      std::vector<cv::Moments> mu(big_contours.size());
      for (long unsigned int i = 0; i < big_contours.size(); i++)
      {
        mu[i] = cv::moments(big_contours[i], false);
      }

      std::vector<cv::Point2f> mc(big_contours.size());
      for(long unsigned int i = 0; i < big_contours.size(); i++)
      {
        mc[i] = cv::Point2f(mu[i].m10/mu[i].m00 , mu[i].m01/mu[i].m00); 
      }

      // Draw contours on color image
      cv::Mat drawing = color_img.clone();

      cv::Scalar red(255, 0, 0);
      cv::Scalar orange(255, 128, 0);
      cv::Scalar yellow(255, 255, 0);
      cv::Scalar green(0, 255, 0);
      cv::Scalar blue(0, 0, 255);
      cv::Scalar indigo(128, 0, 255);
      cv::Scalar violet(255, 0, 255);

      std::vector<cv::Scalar> colors;
      colors.push_back(red);
      colors.push_back(orange);
      colors.push_back(yellow);
      colors.push_back(green);
      colors.push_back(blue);
      colors.push_back(indigo);
      colors.push_back(violet);

      for (long unsigned int i = 0; i < big_contours.size(); i++)
      {
        cv::drawContours(drawing, big_contours, i, colors[i % 6], cv::LINE_AA, 8);
        cv::circle(drawing, mc[i], 20, black, -1, 8, 0);
      }

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

    void track_contours()
    {
      // std::string trackingAlg = "KCF";

      // // cv::MultiTracker tracker = new cv::MultiTracker();
      // cv::legacy::MultiTracker tracker;
      // *tracker = cv::legacy::TrackerKCF::create();

      // // Container of tracked objects
      // std::vector<cv::Rect2d> objects;

      // std::vector<cv::Rect> ROIs;
      // cv::Rect rect1;
      // rect1.x = 1000;
      // rect1.y = 1000;
      // rect1.width = 200;
      // rect1.height = 200;
      // ROIs.push_back(rect1);

      // // Define regions of interest/bounding boxes (ROIs)

      // // Initialize the tracker
      // if (initialize == true)
      // {
      //   std::vector<cv::legacy::Tracker> algorithms;

      //   for (long unsigned int i = 0; i < ROIs.size(); i++)
      //   {
      //     // algorithms.push_back(cv::createTrackerByName(trackingAlg));
      //     // algorithms.push_back(cv::TrackerKCF::create());
      //     algorithms.push_back(tracker);
      //     objects.push_back(ROIs[i]);
      //   }

      //   tracker.add(*algorithms, mask, objects);
      // }
      // else
      // {
      //   //update the tracking result
      //   tracker.update(mask);

      //   cv::Mat tracking_img = color_img.clone();
    
      //   // draw the tracked object
      //   for(unsigned i = 0; i < tracker.getObjects().size(); i++)
      //     cv::rectangle(tracking_img, tracker.getObjects()[i], cv::Scalar(0, 255, 0 ), 2, 1);

      //   // Publish tracking image
      //   std_msgs::msg::Header tracking_header;
      //   tracking_header.stamp = this->get_clock()->now();
      //   tracking_header.frame_id = "camera";
      //   sensor_msgs::msg::Image tracking_pub_msg;
      //   cv_bridge::CvImage tracking_cv_img;
      //   tracking_cv_img = cv_bridge::CvImage(tracking_header, sensor_msgs::image_encodings::RGB8, tracking_img);
      //   tracking_cv_img.toImageMsg(tracking_pub_msg);
      //   tracking_pub_->publish(tracking_pub_msg);
    }
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Vision>());
  rclcpp::shutdown();
  return 0;
}