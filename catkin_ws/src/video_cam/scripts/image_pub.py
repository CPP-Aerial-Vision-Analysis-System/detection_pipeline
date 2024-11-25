#!/usr/bin/env python3

import rospy
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

publisherNodeName = 'camera_sensor_publisher'
topicName = 'image_raw'

rospy.init_node(publisherNodeName, anonymous = True)
publisher = rospy.Publisher(topicName, Image, queue_size=60)
rate = rospy.Rate(30)

videoCaptureObject=cv2.VideoCapture(0)
bridgeObject=CvBridge()

while not rospy.is_shutdown():
    returnValue, capturedFrame = videoCaptureObject.read()
    if returnValue == True:
        rospy.loginfo("Video frame captured and published")
        imageToTransmit = bridgeObject.cv2_to_imgmsg(capturedFrame, encoding='bgr8')
        publisher.publish(imageToTransmit)
    rate.sleep()
