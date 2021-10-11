#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <ros/package.h>
#include <ros/ros.h>
#include <sensor_msgs/image_encodings.h>
#include <std_msgs/String.h>

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

int clamp(int val, int low, int high)
{
  return std::max(low, std::min(val, high));
}
cv::Vec3b rgbclamp(int g, int b, int r)
{
  return cv::Vec3b(clamp(g, 0, 255), clamp(b, 0, 255), clamp(r, 0, 255));
}
cv::Vec3b hsvclamp(int h, int s, int v)
{
  return cv::Vec3b(h % 180, clamp(s, 0, 255), clamp(v, 0, 255));
}

class MyCamera
{
private:
  image_transport::Subscriber image_sub_;
  cv::Mat img_;

  ros::Time t_start_;

public:
  MyCamera()
  {
    ros::NodeHandle n;
    image_transport::ImageTransport it(n);
    image_sub_ = it.subscribe("/camera/color/image_raw", 2, &MyCamera::image_callback, this,
                              image_transport::TransportHints("compressed"));
    t_start_ = ros::Time::now();
  }

  void image_callback(const sensor_msgs::ImageConstPtr& rgb_msg)
  {
    // Load RGB messages
    cv_bridge::CvImagePtr rgb_ptr;
    try
    {
      rgb_ptr = cv_bridge::toCvCopy(rgb_msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }
    img_ = rgb_ptr->image.clone();
  }

  double get_time()
  {
    return (ros::Time::now() - t_start_).toSec();
  }

  cv::Mat get_img()
  {
    return img_;
  }

  void end()
  {
    ros::shutdown();
  }
};

///////////////////////////////////////////////////
int main(int argc, char** argv)
{
  // ROS node
  ros::init(argc, argv, "enshu2_zoom");
  MyCamera camera;

  // Load background image
  std::string package_path = ros::package::getPath(std::string("enshu2"));
  cv::Mat bg_org = cv::imread(package_path + "/img/ut_bg.jpg");
  if (bg_org.empty())
  {
    ROS_ERROR("Couldn't read image!");
    return -1;
  }

  // Main loop
  ros::Rate rate(10);
  while (ros::ok())
  {
    cv::Mat img = camera.get_img();
    if (!img.empty())
    {
      cv::Mat bg_img;
      cv::resize(bg_org, bg_img, img.size());
      cv::Mat org = img.clone();
      int width = img.cols;
      int height = img.rows;
      double center_i = width / 2.0;
      double center_j = height / 2.0;

      // Convert rgb color into hsv color
      cv::cvtColor(org, img, cv::COLOR_BGR2HSV);
      cv::cvtColor(bg_img, bg_img, cv::COLOR_BGR2HSV);
      for (int j = 0; j < height; j++)
      {
        for (int i = 0; i < width; i++)
        {
          // i: 横インデックス，j:縦インデックス
          // [hue, saturation, value(brightness)]
          // hue: 0, ..., 179
          // saturation, value (8 bit): 0, ..., 255
          int h = img.at<cv::Vec3b>(j, i)[0];
          int s = img.at<cv::Vec3b>(j, i)[1];
          int v = img.at<cv::Vec3b>(j, i)[2];

          // Background image value
          int h_bg = bg_img.at<cv::Vec3b>(j, i)[0];
          int s_bg = bg_img.at<cv::Vec3b>(j, i)[1];
          int v_bg = bg_img.at<cv::Vec3b>(j, i)[2];

          /// <write your code>
          bool is_background = false;
          if (is_background)
          {
            v = 0;
          }
          /// </write your code>

          // Write color in the image
          img.at<cv::Vec3b>(j, i) = hsvclamp(h, s, v);
        }
      }
      // Conver hsv color into rgb color
      cv::cvtColor(img, img, cv::COLOR_HSV2BGR);
      cv::cvtColor(bg_img, bg_img, cv::COLOR_HSV2BGR);
      cv::imshow("img", img);
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