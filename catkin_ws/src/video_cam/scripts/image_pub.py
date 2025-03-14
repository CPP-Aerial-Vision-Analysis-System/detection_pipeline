#!/usr/bin/env python3

import rospy
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

publisherNodeName = 'camera_sensor_publisher'
topicName = 'image_raw'

rospy.init_node(publisherNodeName, anonymous = True)
publisher = rospy.Publisher(topicName, Image, queue_size=1)
rate = rospy.Rate(30)

pipeline = (
    "nvarguscamerasrc ! "
    "video/x-raw(memory:NVMM), width=1280, height=720, format=NV12, framerate=30/1 ! "
    "nvvidconv flip-method=0 ! "
    "video/x-raw, format=BGRx ! "
    "videoconvert ! "
    "video/x-raw, format=BGR ! appsink drop=true max-buffers=1 sync=false"
)


# videoCaptureObject=cv2.VideoCapture(pipeline, cv2.CAP_GSTREAMER)
videoCaptureObject=cv2.VideoCapture(0)


bridgeObject=CvBridge()

while not rospy.is_shutdown():
    returnValue, capturedFrame = videoCaptureObject.read()
    if returnValue == True:
        rospy.loginfo_once("Begun Camera Frame Publishing")
        imageToTransmit = bridgeObject.cv2_to_imgmsg(capturedFrame, encoding='bgr8')
        publisher.publish(imageToTransmit)
    rate.sleep()
