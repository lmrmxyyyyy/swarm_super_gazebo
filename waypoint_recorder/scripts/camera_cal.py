#!/usr/bin/env python3
# coding=utf-8

import rospy
import message_filters
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import os

class StereoImageSaver:
    def __init__(self):
        rospy.init_node('stereo_image_saver', anonymous=True)

        self.bridge = CvBridge()
        self.left_image = None
        self.right_image = None

        self.left_image_sub = message_filters.Subscriber('/usb_cam/image_raw', Image)
        self.right_image_sub = message_filters.Subscriber('/usb_cam/image_raw', Image)

        self.ts = message_filters.ApproximateTimeSynchronizer([self.left_image_sub, self.right_image_sub], queue_size=10, slop=0.1)
        self.ts.registerCallback(self.image_callback)

        self.left_image_dir = os.path.join(os.path.dirname(__file__), 'left_images')
        self.right_image_dir = os.path.join(os.path.dirname(__file__), 'right_images')
        os.makedirs(self.left_image_dir, exist_ok=True)
        os.makedirs(self.right_image_dir, exist_ok=True)

        self.image_index = 0

        rospy.loginfo("Press 'p' to save synchronized images.")
    
    def image_callback(self, left_msg, right_msg):
        try:
            # self.left_image = self.bridge.imgmsg_to_cv2(left_msg, desired_encoding='mono8')
            # self.right_image = self.bridge.imgmsg_to_cv2(right_msg, desired_encoding='mono8')

            self.left_image = self.bridge.imgmsg_to_cv2(left_msg, desired_encoding='bgr8')
            self.right_image = self.bridge.imgmsg_to_cv2(right_msg, desired_encoding='bgr8')
        except Exception as e:
            rospy.logerr(f"Failed to convert image: {e}")

    def run(self):
        rate = rospy.Rate(30)  # GUI刷新率

        while not rospy.is_shutdown():
            if self.left_image is not None and self.right_image is not None:
                cv2.imshow("Left Image", self.left_image)
                cv2.imshow("Right Image", self.right_image)

                key = cv2.waitKey(1) & 0xFF
                if key == ord('p'):
                    left_path = os.path.join(self.left_image_dir, f"left_{self.image_index}.png")
                    right_path = os.path.join(self.right_image_dir, f"right_{self.image_index}.png")
                    cv2.imwrite(left_path, self.left_image)
                    cv2.imwrite(right_path, self.right_image)
                    rospy.loginfo(f"Saved images: {left_path}, {right_path}")
                    self.image_index += 1
            rate.sleep()

        cv2.destroyAllWindows()

if __name__ == '__main__':
    try:
        saver = StereoImageSaver()
        saver.run()
    except rospy.ROSInterruptException:
        pass
