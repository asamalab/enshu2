#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <ros/package.h>
#include <ros/ros.h>
#include <sensor_msgs/image_encodings.h>
#include <std_msgs/String.h>

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include "enshu2/utils.hpp"
const double DEPTH_MAX = 6000;
const double DEPTH_MIN = 400;

class MyCamera
{
private:
  image_transport::Subscriber image_sub_;
  image_transport::Subscriber depth_sub_;
  cv::Mat img_;
  cv::Mat depth_;

  ros::Time t_start_;
  bool is_finished_;

public:
  MyCamera()
  {
    ros::NodeHandle n;
    image_transport::ImageTransport it(n);
    image_sub_ = it.subscribe("/camera/color/image_raw", 2, &MyCamera::image_callback, this,
                              image_transport::TransportHints("compressed"));
    depth_sub_ = it.subscribe("/camera/depth/image_rect_raw", 2, &MyCamera::depth_callback, this);
    t_start_ = ros::Time::now();
    is_finished_ = false;
  }

  void image_callback(const sensor_msgs::ImageConstPtr &rgb_msg)
  {
    // Load RGB messages
    cv_bridge::CvImagePtr rgb_ptr;
    try
    {
      rgb_ptr = cv_bridge::toCvCopy(rgb_msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception &e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }
    img_ = rgb_ptr->image.clone();
  }

  void depth_callback(const sensor_msgs::ImageConstPtr &depth_msg)
  {
    // Load Depth messages
    cv_bridge::CvImagePtr depth_ptr;
    try
    {
      depth_ptr = cv_bridge::toCvCopy(depth_msg, sensor_msgs::image_encodings::TYPE_16UC1);
    }
    catch (cv_bridge::Exception &e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }
    if (!img_.empty())
    {
      depth_ = depth_ptr->image.clone();
      for (int j = 0; j < depth_.rows; j++)
      {
        for (int i = 0; i < depth_.cols; i++)
        {
          // Undetected value is recognized as zero
          if (depth_.at<unsigned short>(j, i) < 400 || depth_.at<unsigned short>(j, i) > DEPTH_MAX)
          {
            depth_.at<unsigned short>(j, i) = DEPTH_MAX;
          };
        }
      }

      cv::resize(depth_, depth_, img_.size(), cv::INTER_NEAREST);
    }
  }

  double get_time()
  {
    return (ros::Time::now() - t_start_).toSec();
  }

  cv::Mat get_img()
  {
    return img_;
  }

  cv::Mat get_depth()
  {
    return depth_;
  }

  void end()
  {
    is_finished_ = true;
  }

  bool is_finished()
  {
    return is_finished_;
  }
};

///////////////////////////////////////////////////
int main(int argc, char **argv)
{
  // ROS node
  ros::init(argc, argv, "enshu2_zoom");
  MyCamera camera;

  // Load background image
  std::string package_path = ros::package::getPath(std::string("enshu2"));
  const cv::Mat bg_org = cv::imread(package_path + "/img/ut_bg.jpg");
  if (bg_org.empty())
  {
    ROS_ERROR("Couldn't read image!");
    return -1;
  }

  int blur_size = 21;  // odd
  // Main loop
  ros::Rate rate(10);
  while (ros::ok() && !camera.is_finished())
  {
    cv::Mat img = camera.get_img();
    cv::Mat depth = camera.get_depth();

    if (!img.empty() && !depth.empty())
    {
      cv::Mat bg_img;
      cv::resize(bg_org, bg_img, img.size());
      cv::Mat org = img.clone();
      int width = img.cols;
      int height = img.rows;
      double center_i = width / 2.0;
      double center_j = height / 2.0;
      int center_depth = depth.at<unsigned short>(center_j, center_i);
      ROS_INFO("center depth: %d [m]", center_depth);

      //ぼかし画像
      cv::Mat blurred_img;
      cv::blur(img, blurred_img, cv::Size(blur_size, blur_size));

      for (int j = 0; j < height; j++)
      {
        for (int i = 0; i < width; i++)
        {
          // i: 横インデックス，j:縦インデックス

          int b = img.at<cv::Vec3b>(j, i)[0];
          int g = img.at<cv::Vec3b>(j, i)[1];
          int r = img.at<cv::Vec3b>(j, i)[2];

          int shifted_value = 0;
          int shifted_i = i + shifted_value;

          // 距離 [mm]
          double distance = depth.at<unsigned short>(j, clamp(shifted_i, 0, width - 1));

          /// <write your code>
          bool is_background = false;

          if (is_background)
          {
          }

          ////////////////////

          img.at<cv::Vec3b>(j, i) = rgbclamp(b, g, r);
        }
      }

      cv::imshow("img", img);
      cv::Mat depth_viz;
      depth.convertTo(depth_viz, CV_8U, 255 * 1.0 / DEPTH_MAX);
      cv::imshow("depth", depth_viz);
      if (cv::waitKey(10) == 27)
      {
        break;
      }
    }
    ros::spinOnce();
    rate.sleep();
  }

  return 0;
}
