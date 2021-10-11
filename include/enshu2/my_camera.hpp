#pragma once
#include <cv_bridge/cv_bridge.h>
#include <enshu_msgs/HandResult.h>
#include <ros/ros.h>
#include <sensor_msgs/image_encodings.h>

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

class MyCamera
{
private:
  ros::Subscriber result_sub_;
  cv::Mat img_;
  // Hand landmark
  // idx: hand index
  // idx*6+0: WRIST, idx*6+1: THUMB_TIP, idx*6+2: INDEX_FINGER_TIP
  // idx*6+3: MIDDLE_FINGER_TIP, idx*6+4: RING_FINGER_TIP, idx*6+5: PINKY_TIP
  std::vector<cv::Point2i> hand_lm_;
  // Process hand_lm_ into left_lm_ and right_lm_
  std::vector<cv::Point2i> left_lm_;
  std::vector<cv::Point2i> right_lm_;
  const int LANDMARK_NUM = 6;
  std::vector<cv::Scalar> lm_color_;

  ros::Time t_start_;

  void process_landmark()
  {
    left_lm_.clear();
    right_lm_.clear();

    bool is_right;
    for (int i = 0; i < hand_lm_.size(); i++)
    {
      // Decide right or left based on location of wrist
      if (i % LANDMARK_NUM == 0)
      {
        if (hand_lm_[i].x >= img_.cols / 2)
        {
          is_right = true;
        }
        else
        {
          is_right = false;
        }
      }

      if (is_right)
      {
        right_lm_.push_back(hand_lm_[i]);
      }
      else
      {
        left_lm_.push_back(hand_lm_[i]);
      }
    }
  }

public:
  MyCamera()
  {
    ros::NodeHandle n;
    result_sub_ = n.subscribe("/hand_results", 2, &MyCamera::result_callback, this);
    ROS_INFO("Subscribe hand detection results: /hand_results");
    t_start_ = ros::Time::now();

    lm_color_.clear();
    lm_color_.push_back(cv::Scalar(0, 0, 0));
    lm_color_.push_back(cv::Scalar(200, 0, 0));
    lm_color_.push_back(cv::Scalar(0, 200, 0));
    lm_color_.push_back(cv::Scalar(200, 200, 0));
    lm_color_.push_back(cv::Scalar(0, 0, 200));
    lm_color_.push_back(cv::Scalar(200, 0, 200));
  }

  void result_callback(const enshu_msgs::HandResultConstPtr& msg)
  {
    // Load RGB messages
    cv_bridge::CvImagePtr rgb_ptr;
    try
    {
      rgb_ptr = cv_bridge::toCvCopy(msg->img, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }
    img_ = rgb_ptr->image.clone();

    // Load Landmark info
    hand_lm_.clear();
    for (int i = 0; i < msg->lm.size(); i++)
    {
      cv::Point2i pt(msg->lm[i].x, msg->lm[i].y);
      hand_lm_.push_back(pt);
      // Draw landmark on image
      cv::circle(img_, pt, 8, lm_color_[i % LANDMARK_NUM], 4);
    }
    process_landmark();
  }

  double get_time()
  {
    return (ros::Time::now() - t_start_).toSec();
  }

  cv::Mat get_img()
  {
    return img_;
  }

  int get_num_hands()
  {
    return hand_lm_.size() / LANDMARK_NUM;
  }

  bool has_right_hand()
  {
    return right_lm_.size() != 0;
  }

  bool has_left_hand()
  {
    return left_lm_.size() != 0;
  }

  std::vector<cv::Point2i> get_lefthand_lm()
  {
    return left_lm_;
  }

  std::vector<cv::Point2i> get_righthand_lm()
  {
    return right_lm_;
  }

  bool has_1st_hand()
  {
    return get_num_hands() > 0;
  }

  bool has_2nd_hand()
  {
    return get_num_hands() > 1;
  }

  std::vector<cv::Point2i> get_1sthand_lm()
  {
    std::vector<cv::Point2i> lm;
    if (!has_1st_hand())
    {
      return lm;
    }

    for (int i = 0; i < LANDMARK_NUM; i++)
    {
      lm.push_back(hand_lm_[i]);
    }
    return lm;
  }

  std::vector<cv::Point2i> get_2ndhand_lm()
  {
    std::vector<cv::Point2i> lm;
    if (!has_2nd_hand())
    {
      return lm;
    }

    for (int i = 0; i < LANDMARK_NUM; i++)
    {
      lm.push_back(hand_lm_[LANDMARK_NUM + i]);
    }
    return lm;
  }
};
