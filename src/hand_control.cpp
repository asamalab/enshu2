#include <ros/ros.h>

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include "enshu2/gesture_camera.hpp"
#include "enshu2/myrobot.h"

///////////////////////////////////////////////////
int main(int argc, char** argv)
{
  // ROS node
  ros::init(argc, argv, "enshu2_control");
  GestureCamera camera;
  MyRobot robot;

  // Font setting
  std::string text_contents;
  cv::Point2i font_loc;
  int font_baseline;
  int FONT = cv::FONT_HERSHEY_SIMPLEX;
  double FONT_SIZE = 0.8;
  double FONT_THICKNESS = 2;
  cv::Scalar FONT_RED = cv::Scalar(0, 0, 255);
  cv::Scalar FONT_GREEN = cv::Scalar(0, 255, 0);
  cv::Scalar FONT_BLUE = cv::Scalar(255, 0, 0);
  cv::Scalar FONT_BLACK = cv::Scalar(0, 0, 0);
  cv::Scalar FONT_WHITE = cv::Scalar(255, 255, 255);

  // Main loop
  ros::Rate rate(30);
  while (ros::ok())
  {
    cv::Mat img = camera.get_img();
    if (!img.empty())
    {
      int width = img.cols;
      int height = img.rows;
      double center_i = width / 2.0;
      double center_j = height / 2.0;

      // 手首：left_lms[0], right_lms[0]
      // 親指：left_lms[1], right_lms[1]
      // 人差し指：left_lms[2], right_lms[2]
      // 中指：left_lms[3], right_lms[3]
      // 薬指：left_lms[4], right_lms[4]
      // 小指：left_lms[5], right_lms[5]
      std::vector<cv::Point2i> left_lms = camera.get_lefthand_lm();
      std::vector<cv::Point2i> right_lms = camera.get_righthand_lm();
      // 左右問わず，１つ目の手の検出ランドマーク
      std::vector<cv::Point2i> first_lms = camera.get_1sthand_lm();
      // 左右問わず，２つ目の手の検出ランドマーク
      std::vector<cv::Point2i> second_lms = camera.get_2ndhand_lm();

      /// <write your code>

      // Command for robot
      double v = 0;
      double omega = 0;

      if (camera.has_left_hand())
      {
        if (left_lms[0].y < center_j)
        {
          v = 0.05;
        }
        else
        {
          v = -0.05;
        }
      }

      if (camera.has_right_hand())
      {
        if (right_lms[0].y < center_j)
        {
          omega = 0.2;
        }
        else
        {
          omega = -0.2;
        }
      }
      if (!camera.has_left_hand() && !camera.has_right_hand())
      {
        v = 0.0;
        omega = 0.0;
      }

      // Send command to robot
      robot.move(v, omega);

      // if (camera.has_1st_hand())
      // {
      //   std::cout << first_lms[0] << ",";
      //   if (camera.has_2nd_hand())
      //   {
      //     std::cout << second_lms[0];
      //   }
      //   std::cout << std::endl;
      // }

      /// </write your code>

      ///////////////////////////////////////
      // Command
      std::ostringstream ostr;
      ostr.precision(3);
      ostr << "[v:" << v << ", omega:" << omega << "]";
      text_contents = ostr.str();
      int baseline;
      cv::Size font_size = cv::getTextSize(text_contents, FONT, 1.5 * FONT_SIZE, 3, &baseline);
      font_loc = cv::Point2i(center_i - font_size.width / 2, center_j + font_size.height / 2 + baseline);
      cv::putText(img, text_contents, font_loc, FONT, 1.5 * FONT_SIZE, cv::Scalar(255, 255, 255), FONT_THICKNESS);

      // resize for visualization
      cv::resize(img, img, cv::Size(), 2, 2);
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
