#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Image
from geometry_msgs.msg import Point
from enshu_msgs.msg import HandResult
from cv_bridge import CvBridge
import cv2
import numpy as np

import mediapipe as mp

mp_drawing = mp.solutions.drawing_utils
mp_drawing_styles = mp.solutions.drawing_styles
mp_hands = mp.solutions.hands


def main():
    rospy.init_node("enshu2_py")

    hands = mp_hands.Hands(min_detection_confidence=0.5, min_tracking_confidence=0.5)
    rospy.loginfo("Setup mediapipe hand")

    cap = cv2.VideoCapture(0)
    if cap.isOpened():
        rospy.loginfo("Open camera")
    else:
        rospy.logerr("Cannot open camera")
    result_pub = rospy.Publisher("/hand_results", HandResult, queue_size=2)
    rospy.loginfo("Publishing hand detection info: /hand_results")

    bridge = CvBridge()
    r = rospy.Rate(15)
    while not rospy.is_shutdown() and cap.isOpened():
        ret, img = cap.read()
        if not ret:
            continue

        # Process image
        img = cv2.flip(img, 1)

        # Prepare ROS message
        msg = HandResult()
        msg.img = bridge.cv2_to_imgmsg(img, "bgr8")

        # To improve performance, optionally mark the image as not writeable to
        # pass by reference.
        img.flags.writeable = False
        img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
        results = hands.process(img)

        if results.multi_hand_landmarks:
            height = img.shape[0]
            width = img.shape[1]
            for hand_landmarks in results.multi_hand_landmarks[::-1]:
                # 0: WRIST, 4: THUMB_TIP, 8: INDEX_FINGER_TIP
                # 12: MIDDLE_FINGER_TIP, 16: RING_FINGER_TIP, 20: PINKY_TIP
                for idx in [0, 4, 8, 12, 16, 20]:
                    lm = hand_landmarks.landmark[idx]
                    msg.lm.append(Point(x=int(lm.x * width), y=int(lm.y * height)))
        result_pub.publish(msg)
        r.sleep()

    hands.close()
    rospy.loginfo("Finish hand detection node")


if __name__ == "__main__":
    main()
