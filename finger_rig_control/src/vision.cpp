/*******************************************************************************
* VISION NODE
* 
* Node Description:
* This node handles the capture and analysis of images from an Allied Vision camera
* looking upwards at the artifical finger tip and its embedded fiducial pattern.
* 
* Publishers:
* /camera/image_gray        - standard color image from camera converted to grayscale
* /camera/image_normalized  - grayscale image normalized
* /camera/image_finger      - grayscale image cropped to only show area of fingertip
* /camera/image_mask        - binary image of fiducials on finger cropped to area of fingertip
* /camera/image_blur        - blurred image useful for certain CV operations
* /camera/image_contours    - color image overlaid with contour traces and centroids
* /camera/image_tracking    - mask image with tracking bounding boxes over fiducials
* 
* Subscribers:
* /camera/image             - standard color image published by Allied Vision camera
* 
* Services:
* None
******************************************************************************/

#include <rclcpp/rclcpp.hpp>

#include <std_srvs/srv/empty.hpp>
#include <std_msgs/msg/header.h>
#include <sensor_msgs/msg/image.hpp>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/objdetect/objdetect.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgcodecs/imgcodecs.hpp>
#include <opencv2/tracking.hpp>
#include <opencv2/tracking/tracking_legacy.hpp>
#include <opencv2/tracking/tracking.hpp>
#include <opencv2/videoio/videoio.hpp>
#include <opencv2/core/utility.hpp>

using std::placeholders::_1, std::placeholders::_2;

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
      finger_pub_ = this->create_publisher<sensor_msgs::msg::Image>("/camera/image_finger", 10);
      mask_pub_ = this->create_publisher<sensor_msgs::msg::Image>("/camera/image_mask", 10);
      blur_pub_ = this->create_publisher<sensor_msgs::msg::Image>("/camera/image_blur", 10);
      contour_pub_ = this->create_publisher<sensor_msgs::msg::Image>("/camera/image_contours", 10);
      tracking_pub_ = this->create_publisher<sensor_msgs::msg::Image>("/camera/image_tracking", 10);
    }

  private:
    // Initialize subscribers
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;

    // Initialize publishers
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr gray_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr norm_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr finger_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr mask_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr blur_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr contour_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr tracking_pub_;

    // Initialize variables
    cv_bridge::CvImagePtr cv_img_ptr;
    cv::Mat color_img;
    cv::Mat gray_img;
    cv::Mat cropped_gray_img;
    cv::Mat finger_mask;
    cv::Mat mask;
    cv::Mat mask_bgr;

    std::vector<cv::Point2f> centroids;
    std::vector<cv::Point2f> old_centroids;
    std::vector<cv::Rect> bboxes;
    std::vector<cv::Point> centers;
    cv::Ptr<cv::legacy::tracking::MultiTracker> multiTracker;
    cv::Rect ex_rect, ex_rect2;

    bool first_time = true;

    /*!
      Subscribes to /camera/image topic from Allied Vision camera, converts to grayscale,
      and publishes the grayscale image. Also runs the relevant functions from this node. 
    */
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

      // Publish grayscale image
      std_msgs::msg::Header gray_header;
      gray_header.stamp = this->get_clock()->now();
      gray_header.frame_id = "camera";
      sensor_msgs::msg::Image gray_pub_msg;
      cv_bridge::CvImage gray_cv_img;
      gray_cv_img = cv_bridge::CvImage(gray_header, sensor_msgs::image_encodings::MONO8, gray_img);
      gray_cv_img.toImageMsg(gray_pub_msg);
      gray_pub_->publish(gray_pub_msg);

      // Run relevant functions
      find_finger();
      threshold_img();
      find_contours();
      make_fiducial_circles();
      // track_fiducials_opencv();  // tried unsuccessfully to use OpenCV MultiTracker
      track_fiducials_custom();
    }

    /*!
      Detects contours of finger tip in image using thresholding, determines the centroid of this contour,
      then creates a cropped version of the grayscale image that only shows information within the 
      bounds of the finger tip contour (ideally, only the finger tip fiducials).

      This worked under ideal conditions, but occasionally would switch the crop to a random location in frame
      where it was detecting a different contour centroid. Opted for a static circle crop of approximately the size
      of the finger tip at the center of the frame.
    */
    void find_finger()
    {
      // Threshold gray image
      cv::threshold(gray_img, finger_mask, 210, 255, CV_THRESH_BINARY);

      // Dilate and erode to make lines around finger stronger 
      int morph_size = 8;
      cv::Mat element = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(2 * morph_size + 1, 2 * morph_size + 1), cv::Point(morph_size, morph_size));
      cv::dilate(finger_mask, finger_mask, element, cv::Point(-1, -1), 5, 1, 1);
      cv::erode(finger_mask, finger_mask, element, cv::Point(-1, -1), 5, 1, 1);

      // Find contours in finger_mask
      cv::Mat contour_output = finger_mask.clone();
      std::vector<std::vector<cv::Point>> finger_contours;
      std::vector<cv::Vec4i> finger_hierarchy;
      cv::findContours(contour_output, finger_contours, finger_hierarchy, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_NONE);

      // Filter out any small contours (finger contour should be large)
      std::vector<std::vector<cv::Point>> final_contour;
      for (long unsigned int i = 0; i < finger_contours.size(); i++)
      {
        if (cv::contourArea(finger_contours[i]) > 15000)
        {
          final_contour.push_back(finger_contours[i]);
        }
      }

      // Find centroid of finger
      std::vector<cv::Moments> mu(final_contour.size());
      for (long unsigned int i = 0; i < final_contour.size(); i++)
      {
        mu[i] = cv::moments(final_contour[i], false);
      }

      std::vector<cv::Point2f> finger_centroid(final_contour.size());
      for(long unsigned int i = 0; i < final_contour.size(); i++)
      {
        finger_centroid[i] = cv::Point2f(mu[i].m10/mu[i].m00 , mu[i].m01/mu[i].m00); 
      }

      // Draw contour
      cv::Mat finger_show = color_img.clone();

      // Create black image with white circle to use for cropping finger circle out of color image
      // Type is CV_8UC3, 8 = bits per pixel, U = unsigned int (0-255), C3 = 3 channels per pixel
      cv::Mat circle_mask = cv::Mat::zeros(cv::Size(finger_show.cols, finger_show.rows), CV_8UC3);
      cv::Point center;
      center.x = 1296;
      center.y = 972;

      // Using circle at detected centroid of finger contour was buggy, so switched to manual circle 
      // at center of frame
      cv::Scalar white(255, 255, 255);
      // cv::circle(circle_mask, finger_centroid[0], 920, white, -1);
      cv::circle(circle_mask, center, 960, white, -1);

      // Crop color image (finger_show) to only include area where fingertip is
      cv::bitwise_and(circle_mask, finger_show, finger_show);

      // Convert cropped image of finger to grayscale
      cv::cvtColor(finger_show, cropped_gray_img, cv::COLOR_RGB2GRAY);

      // Publish finger image
      std_msgs::msg::Header finger_header;
      finger_header.stamp = this->get_clock()->now();
      finger_header.frame_id = "camera";
      sensor_msgs::msg::Image finger_pub_msg;
      cv_bridge::CvImage finger_cv_img;
      finger_cv_img = cv_bridge::CvImage(finger_header, sensor_msgs::image_encodings::MONO8, cropped_gray_img);
      finger_cv_img.toImageMsg(finger_pub_msg);
      finger_pub_->publish(finger_pub_msg);
    }

    /*!
      Thresholds gray image to highlight fiducial pattern, then publishes this mask
      image.
    */
    void threshold_img()
    {
      // For paper test fiducials
      // cv::threshold(cropped_gray_img, mask, 170, 255, CV_THRESH_BINARY);

      // For real fingertip fiducials
      cv::threshold(cropped_gray_img, mask, 165, 255, CV_THRESH_BINARY);

      // Invert threshold mask
      cv::bitwise_not(mask, mask);

      // Dilation and erosion to tweak mask image in order to clean up fiducials
      int morph_size = 3;
      cv::Mat element = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(2 * morph_size + 1, 2 * morph_size + 1), cv::Point(morph_size, morph_size));
      cv::erode(mask, mask, element, cv::Point(-1, -1), 2, 1, 1);

      // If using paper test fiducials, comment out this dilate and erode
      morph_size = 7;
      element = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(2 * morph_size + 1, 2 * morph_size + 1), cv::Point(morph_size, morph_size));
      cv::dilate(mask, mask, element, cv::Point(-1, -1), 4, 1, 1);
      cv::erode(mask, mask, cv::Mat(), cv::Point(-1, -1), 4, 1, 1);

      // Create BGR version of mask for use with tracking bounding boxes
      cv::cvtColor(mask, mask_bgr, cv::COLOR_GRAY2BGR);

      // // Publish mask image
      std_msgs::msg::Header header;
      header.stamp = this->get_clock()->now();
      header.frame_id = "camera";
      sensor_msgs::msg::Image mask_pub_msg;
      cv_bridge::CvImage mask_cv_img;
      mask_cv_img = cv_bridge::CvImage(header, sensor_msgs::image_encodings::MONO8, mask);
      mask_cv_img.toImageMsg(mask_pub_msg);
      mask_pub_->publish(mask_pub_msg);
    }

    /*!
      Detects fiducial blobs in mask image, their associated contours, and their centroids.
      Publishes these contours and centroids overlaid on the color_img.
    */
    void find_contours()
    {
      // Find contours in mask
      std::vector<std::vector<cv::Point>> contours;
      std::vector<cv::Vec4i> hierarchy;
      cv::Mat contour_output = mask.clone();
      // cv::findContours(contour_output, contours, hierarchy, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_NONE);
      cv::findContours(contour_output, contours, hierarchy, cv::RETR_TREE, cv::CHAIN_APPROX_NONE);

      // Filter out any small or large contours
      std::vector<std::vector<cv::Point>> big_contours;
      for (long unsigned int i = 0; i < contours.size(); i++)
      {
        // Narrower range for paper fiducial testing
        // if (cv::contourArea(contours[i]) > 5000 && cv::contourArea(contours[i]) < 25000)
        if (cv::contourArea(contours[i]) > 2000 && cv::contourArea(contours[i]) < 35000)
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

      centroids.clear();
      
      for(long unsigned int i = 0; i < big_contours.size(); i++)
      {
        centroids.push_back(cv::Point2f(mu[i].m10/mu[i].m00 , mu[i].m01/mu[i].m00)); 
      }

      // Define some colors and put them in a vector
      cv::Scalar red(255, 0, 0);
      cv::Scalar orange(255, 128, 0);
      cv::Scalar yellow(255, 255, 0);
      cv::Scalar green(0, 255, 0);
      cv::Scalar blue(0, 0, 255);
      cv::Scalar indigo(128, 0, 255);
      cv::Scalar violet(255, 0, 255);
      cv::Scalar black (0, 0, 0);

      std::vector<cv::Scalar> colors;
      colors.push_back(red);
      colors.push_back(orange);
      colors.push_back(yellow);
      colors.push_back(green);
      colors.push_back(blue);
      colors.push_back(indigo);
      colors.push_back(violet);

      // Create copy of color_img
      cv::Mat color_img_w_drawing = color_img.clone();

      // Draw contours and centroid dots on copy of color_img
      for (long unsigned int i = 0; i < big_contours.size(); i++)
      {
        cv::drawContours(color_img_w_drawing, big_contours, i, colors[i % 6], cv::LINE_AA, 8);
        cv::circle(color_img_w_drawing, centroids[i], 20, black, -1, 8, 0);
      }

      // Publish image with contours drawn
      std_msgs::msg::Header contour_header;
      contour_header.stamp = this->get_clock()->now();
      contour_header.frame_id = "camera";
      sensor_msgs::msg::Image contour_pub_msg;
      cv_bridge::CvImage contour_cv_img;
      contour_cv_img = cv_bridge::CvImage(contour_header, sensor_msgs::image_encodings::RGB8, color_img_w_drawing);
      contour_cv_img.toImageMsg(contour_pub_msg);
      contour_pub_->publish(contour_pub_msg);
    }

    /*!
      Test function that attempted to create a custom mask image using perfect
      circles at centroid locations rather than amorphous detected blobs.
    */
    void make_fiducial_circles()
    {

      // cv::Mat fid_mask = cv::Mat::zeros(cv::Size(mask.cols, mask.rows), CV_8UC3);
      // cv::Scalar white(255, 255, 255);

      // for (unsigned long int i = 0; i < centroids.size(); i++)
      // {
      //   cv::circle(fid_mask, centroids[i], 70, white, -1);
      // }

      // Crop color image (finger_show) to only include area where fingertip is
      // cv::bitwise_and(circle_mask, finger_show, finger_show);

      // Convert cropped image of finger to grayscale
      // cv::cvtColor(fid_mask, fid_mask, cv::COLOR_RGB2GRAY);

      // Publish mask image
      // std_msgs::msg::Header header;
      // header.stamp = this->get_clock()->now();
      // header.frame_id = "camera";
      // sensor_msgs::msg::Image mask_pub_msg;
      // cv_bridge::CvImage mask_cv_img;
      // mask_cv_img = cv_bridge::CvImage(header, sensor_msgs::image_encodings::MONO8, fid_mask);
      // mask_cv_img.toImageMsg(mask_pub_msg);
      // mask_pub_->publish(mask_pub_msg);
    }

    /*!
      Tracks centroids of detected fiducial blobs as they move within frame.
      Works very well for paper test fiducials due to constant colors, shape,
      and size. Also works for actual artifical finger tip, but lighting and 
      contrast affect the tracking quite a bit.
    */
    void track_fiducials_custom()
    {
      std::vector<cv::Point2f> new_centroids;
      std::vector<cv::Point2f> placeholder_centroids;

      if (first_time == true)
      {
        RCLCPP_INFO(this->get_logger(), "FIRST TIME");

        old_centroids = centroids;

        for (unsigned long int i = 0; i < old_centroids.size(); i++)
        {
          cv::Rect bbox;
          int bbox_size = 200;
          bbox.x = old_centroids[i].x - bbox_size/2;
          bbox.y = old_centroids[i].y - bbox_size/2;
          bbox.width = bbox_size;
          bbox.height = bbox_size;
          bboxes.push_back(bbox);
        }

        first_time = false;
      }
      else
      {
        // Get updated centroids
        new_centroids = centroids;
        placeholder_centroids = old_centroids;

        // Compare to old ones
        for (unsigned long int i = 0; i < old_centroids.size(); i++)
        {
          double min_dist = 1000000;
          
          for (unsigned long int j = 0; j < new_centroids.size(); j++)
          {
            cv::Point2f diff = old_centroids[i] - new_centroids[j];
            double dist = cv::sqrt(diff.x*diff.x + diff.y*diff.y);

            // If new distance is less than the current minimum found
            if (dist < min_dist)
            {
              min_dist = dist;
              placeholder_centroids[i] = new_centroids[j];
            }
          }
        }

        old_centroids = placeholder_centroids;

        // Update bounding boxes
        for (unsigned long int i = 0; i < old_centroids.size(); i++)
        {
          cv::Rect bbox;
          int bbox_size = 200;
          bbox.x = old_centroids[i].x - bbox_size/2;
          bbox.y = old_centroids[i].y - bbox_size/2;
          bbox.width = bbox_size;
          bbox.height = bbox_size;
          bboxes.push_back(bbox);
        }
      }

      // Create copy of mask in BGR format (so colored boxes can be drawn)
      cv::Mat tracking_img = mask_bgr.clone();

      // Make pretty color vector
      cv::Scalar red(255, 0, 0);
      cv::Scalar orange(255, 128, 0);
      cv::Scalar yellow(255, 255, 0);
      cv::Scalar green(0, 255, 0);
      cv::Scalar blue(0, 0, 255);
      cv::Scalar indigo(128, 0, 255);
      cv::Scalar violet(255, 0, 255);
      cv::Scalar black (0, 0, 0);

      std::vector<cv::Scalar> colors;
      colors.push_back(red);
      colors.push_back(orange);
      colors.push_back(yellow);
      colors.push_back(green);
      colors.push_back(blue);
      colors.push_back(indigo);
      colors.push_back(violet);
      colors.push_back(red);
      colors.push_back(orange);
      colors.push_back(yellow);
      colors.push_back(green);
      colors.push_back(blue);
      colors.push_back(indigo);
      colors.push_back(violet);
      colors.push_back(red);
      colors.push_back(orange);
      colors.push_back(yellow);
      colors.push_back(green);
      colors.push_back(blue);
      colors.push_back(indigo);
      colors.push_back(violet);

      RCLCPP_INFO(this->get_logger(), "NEW SET OF BOUNDING BOXES:");

      // Draw bounding boxes
      for(unsigned long int i = 0; i < bboxes.size(); i++)
      {        
        cv::rectangle(tracking_img, bboxes[i], colors[i], 20);
        RCLCPP_INFO(this->get_logger(), "Centroid %ld:    x: %d   y: %d", i, bboxes[i].x, bboxes[i].y);
      }

      RCLCPP_INFO(this->get_logger(), "");

      // Reset bounding boxes vector after drawing them
      bboxes.clear();

      // Publish tracking image
      std_msgs::msg::Header tracking_header;
      tracking_header.stamp = this->get_clock()->now();
      tracking_header.frame_id = "camera";
      sensor_msgs::msg::Image tracking_pub_msg;
      cv_bridge::CvImage tracking_cv_img;
      tracking_cv_img = cv_bridge::CvImage(tracking_header, sensor_msgs::image_encodings::RGB8, tracking_img);
      tracking_cv_img.toImageMsg(tracking_pub_msg);
      tracking_pub_->publish(tracking_pub_msg);
    }

    /*!
      Attempt at tracking detected fiducials using legacy OpenCV MultiTracker
      object. It was able to track a single object, but opted for creating a
      custom tracker (above) as opposed to trying to debug this.
    */
    void track_fiducials_opencv()
    {
      // // Initialize the tracker
      // if (first_time == true)
      // {
      //   // Create multitracker
      //   multiTracker = cv::legacy::tracking::MultiTracker::create();

      //   int bbox_size = 200;

      //   RCLCPP_INFO(this->get_logger(), "centroids length2: %ld", centroids.size());

      //   for (long unsigned int i = 0; i < centroids.size(); i++)
      //   {
      //     cv::Rect bbox;
      //     bbox.x = centroids[i].x - bbox_size/2;
      //     bbox.y = centroids[i].y - bbox_size/2;
      //     bbox.width = bbox_size;
      //     bbox.height = bbox_size;
      //     bboxes.push_back(bbox);
      //   }

      //   RCLCPP_INFO(this->get_logger(), "bboxes length: %ld", bboxes.size());

      //   // Initialize multitracker
      //   cv::Ptr<cv::legacy::tracking::Tracker> tracker = cv::legacy::tracking::TrackerKCF::create();

      //   for(long unsigned int i=0; i < bboxes.size(); i++)
      //   {
      //     multiTracker->add(tracker, mask_bgr, cv::Rect2d(bboxes[i]));
      //     RCLCPP_INFO(this->get_logger(), "Added tracker to multitracker");
      //   }
      //   first_time = false;
      // }
      // else
      // {
      //   // Update the tracking result
      //   multiTracker->update(mask_bgr);

      //   cv::Mat tracking_img = mask_bgr.clone();

      //   cv::Scalar green(0, 255, 0);
    
      //   // Draw the tracked objects
      //   // RCLCPP_INFO(this->get_logger(), "multiTracker.getobjects.size: %ld", multiTracker->getObjects().size());

      //   for(unsigned i=0; i<multiTracker->getObjects().size(); i++)
      //   {
      //     bboxes[i] = multiTracker->getObjects()[i];

      //     // RCLCPP_INFO(this->get_logger(), "object_x: %f, object_y: %f", multiTracker->getObjects()[i].x, multiTracker->getObjects()[i].y);
          
      //     // cv::rectangle(tracking_img, bboxes[i], cv::Scalar(0, 255, 0), 20);
      //     cv::rectangle(tracking_img, multiTracker->getObjects()[i], cv::Scalar(0, 255, 0), 20);
      //   }

      //   // Publish tracking image
      //   std_msgs::msg::Header tracking_header;
      //   tracking_header.stamp = this->get_clock()->now();
      //   tracking_header.frame_id = "camera";
      //   sensor_msgs::msg::Image tracking_pub_msg;
      //   cv_bridge::CvImage tracking_cv_img;
      //   tracking_cv_img = cv_bridge::CvImage(tracking_header, sensor_msgs::image_encodings::RGB8, tracking_img);
      //   tracking_cv_img.toImageMsg(tracking_pub_msg);
      //   tracking_pub_->publish(tracking_pub_msg);
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