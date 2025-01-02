#!/usr/bin/env python3

import rospy
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge


class ImagePublisher:
    def __init__(self):
        rospy.init_node("cam_pub", anonymous=True)

        self.topic_name = rospy.get_param("~input_topic", "image_raw")
        self.use_cam = rospy.get_param("~camera", True)
        print(type(self.use_cam))
        self.file_path = "/home/car-video.mp4"

        self.publisher = rospy.Publisher(self.topic_name, Image, queue_size=60)
        self.rate = rospy.Rate(30)
        self.bridge = CvBridge()

        if self.use_cam:
            self.video_capture = cv2.VideoCapture(0)
        else:
            self.video_capture = cv2.VideoCapture(self.file_path)

    def publish_image(self):
        while not rospy.is_shutdown():
            ret, frame = self.video_capture.read()
            if ret:
                rospy.loginfo("Video frame captured and published")
                image_msg = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
                self.publisher.publish(image_msg)
            else:
                rospy.logwarn("Failed to capture frame")
            self.rate.sleep()


if __name__ == "__main__":
    try:
        image_publisher = ImagePublisher()
        image_publisher.publish_image()
    except rospy.ROSInterruptException:
        pass
