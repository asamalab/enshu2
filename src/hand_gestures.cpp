#include <ros/ros.h>

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include "enshu2/gesture_camera.hpp"

///////////////////////////////////////////////////
int main(int argc, char** argv)
{
  // ROS node
  ros::init(argc, argv, "enshu2_hand");
  GestureCamera camera;

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

      /// <write your code>
      // 画像の左半分に出てくる手を左の手，
      // 画像の右半分に出てくる手を左の手とする．
      // 手の形からジェスチャーを判断するコードの作成
      // -1: 未検出，0:グー，1:チョキ, 2:パー
      int left_results = -1;
      int right_results = -1;

      // 左の手が検出されている時
      if (camera.has_left_hand())
      {
        std::vector<cv::Point2i> lms = camera.get_lefthand_lm();
        // 手首の検出位置
        cv::Point2i lm0 = lms[0];
        // 親指の検出位置
        cv::Point2i lm1 = lms[1];
        // 人差し指の検出位置
        cv::Point2i lm2 = lms[2];
        // 中指の検出位置
        cv::Point2i lm3 = lms[3];
        // 薬指の検出位置
        cv::Point2i lm4 = lms[4];
        // 小指の検出位置
        cv::Point2i lm5 = lms[5];

        //////////////////////////////////
        // ジェスチャー認識のロジックを記述
        //////////////////////////////////

        // 座標へのアクセス
        int dist_x = lm1.x - lm0.x;
        int dist_y = lm4.y - lm2.y;
        // 2点間のユークリッド距離
        double dist = cv::norm(lm1 - lm3);

        if (dist < 100)
        {
          left_results = 0;
        }
      }

      // 右の手が検出されている時
      if (camera.has_right_hand())
      {
        std::vector<cv::Point2i> lms = camera.get_righthand_lm();
        // 手首の検出位置
        cv::Point2i lm0 = lms[0];
        // 親指の検出位置
        cv::Point2i lm1 = lms[1];
        // 人差し指の検出位置
        cv::Point2i lm2 = lms[2];
        // 中指の検出位置
        cv::Point2i lm3 = lms[3];
        // 薬指の検出位置
        cv::Point2i lm4 = lms[4];
        // 小指の検出位置
        cv::Point2i lm5 = lms[5];

        //////////////////////////////////
        // ジェスチャー認識のロジックを記述
        //////////////////////////////////

        // 座標へのアクセス
        int dist_x = lm1.x - lm0.x;
        int dist_y = lm4.y - lm2.y;
        // 2点間のユークリッド距離
        double dist = cv::norm(lm1 - lm3);

        if (dist < 100)
        {
          right_results = 0;
        }
      }

      // 勝敗を判断するコードの作成
      // -1: 未検出, 0:引き分け, 1:左, 2:右
      int winner = 0;
      if (left_results == -1 || right_results == -1)
      {
        winner = -1;
      }
      else if (left_results == right_results)
      {
        winner = 0;
      }
      else
      {
        winner = 1;
      }
      /// </write your code>

      ///////////////////////////////////////
      // 以下，可視化用のコードです．
      // 修正する必要はありません．
      // Add title in image
      text_contents = "Rock-paper-scissors";
      font_loc = cv::Point2i(20, 30);
      cv::putText(img, text_contents, font_loc, FONT, FONT_SIZE, FONT_BLUE, FONT_THICKNESS);

      // Add recognition result in images
      std::vector<std::string> hand_recog_string = { "Undetected", "Rock", "Scissors", "Paper" };
      // Left result
      text_contents = hand_recog_string[left_results + 1];
      font_loc = cv::Point2i(30, height - 50);
      cv::putText(img, text_contents, font_loc, FONT, 1.5 * FONT_SIZE, cv::Scalar(255, 255, 0), 1.5 * FONT_THICKNESS);
      // Right result
      text_contents = hand_recog_string[right_results + 1];
      font_loc = cv::Point2i(420, height - 50);
      cv::putText(img, text_contents, font_loc, FONT, 1.5 * FONT_SIZE, cv::Scalar(0, 255, 255), 1.5 * FONT_THICKNESS);

      // Final result
      std::vector<std::string> winner_string = { "-", "Draw", "Winner:Left", "Winner:Right" };
      text_contents = winner_string[winner + 1];
      int baseline;
      cv::Size font_size = cv::getTextSize(text_contents, FONT, 2.5 * FONT_SIZE, 3, &baseline);
      font_loc = cv::Point2i(center_i - font_size.width / 2, center_j + font_size.height / 2 + baseline);
      cv::putText(img, text_contents, font_loc, FONT, 2.5 * FONT_SIZE, cv::Scalar(255, 255, 255), 3 * FONT_THICKNESS);

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
