#include <iostream>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include "enshu2/utils.hpp"

int main(int argc, char** argv)
{
  // Load image template
  std::string package_path = "../";
  cv::Mat img = cv::imread(package_path + "/img/img_before.jpg");
  if (img.empty())
  {
    std::cerr << "Couldn't read image!";
    return -1;
  }

  cv::Mat org = img.clone();
  int width = img.cols;
  int height = img.rows;
  double center_i = width / 2.0;
  double center_j = height / 2.0;
  std::cout << "Image:" << height << ", " << width << std::endl;

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
