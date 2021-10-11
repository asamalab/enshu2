#include <ros/package.h>
#include <ros/ros.h>

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

int main(int argc, char** argv)
{
  // ROS node
  ros::init(argc, argv, "enshu2");
  ros::NodeHandle n;

  // Load image template
  std::string package_path = ros::package::getPath(std::string("enshu2"));
  cv::Mat img = cv::imread(package_path + "/img/img_before.jpg");
  if (img.empty())
  {
    ROS_ERROR("Couldn't read image!");
    return -1;
  }

  cv::Mat org = img.clone();
  int width = img.cols;
  int height = img.rows;
  double center_i = width / 2.0;
  double center_j = height / 2.0;
  ROS_INFO("Image: %dx%d", height, width);

  // Convert rgb color into hsv color
  cv::cvtColor(org, img, cv::COLOR_BGR2HSV);
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

      /// <write your code>

      /// </write your code>

      // Write color in the image
      img.at<cv::Vec3b>(j, i) = hsvclamp(h, s, v);
    }
  }
  // Conver hsv color into rgb color
  cv::cvtColor(img, img, cv::COLOR_HSV2BGR);

  // Visualize image
  cv::imshow("original image", org);
  cv::imshow("processed image", img);
  cv::imwrite(package_path + "/img/img_hsv_after.jpg", img);
  cv::waitKey();
  return 0;
}
