#pragma once
#include <algorithm>
#include <opencv2/core.hpp>

int clamp(int val, int low, int high)
{
  return std::max(low, std::min(val, high));
}

cv::Vec3b rgbclamp(int b, int g, int r)
{
  return cv::Vec3b(clamp(b, 0, 255), clamp(g, 0, 255), clamp(r, 0, 255));
}

cv::Vec3b hsvclamp(int h, int s, int v)
{
  return cv::Vec3b(h % 180, clamp(s, 0, 255), clamp(v, 0, 255));
}