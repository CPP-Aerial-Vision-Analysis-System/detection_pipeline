#!/usr/bin/env python3

import rospy
import cv2
from sensor_msgs.msg import Image
from std_msgs.msg import Bool
from cv_bridge import CvBridge
import os, time

publisherNodeName = "camera_sensor_publisher"
topicName = "image_raw"
photo_path = "/detection_pipeline/catkin_ws/src/video_cam/mapping"

if not os.path.exists(photo_path):
    os.makedirs(photo_path)

rospy.init_node(publisherNodeName, anonymous=True)
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
videoCaptureObject = cv2.VideoCapture(0)


bridgeObject = CvBridge()

# flag to indicate when to save photo to disk
capture_photo = False


def camera_trigger_callback(msg):
    global capture_photo
    if msg.data:
        rospy.loginfo("Camera trigger received. Preparing to capture a photo.")
        capture_photo = True


rospy.Subscriber("/camera/trigger", Bool, camera_trigger_callback)

while not rospy.is_shutdown():
    returnValue, capturedFrame = videoCaptureObject.read()
    if returnValue == True:
        rospy.loginfo_once("Begun Camera Frame Publishing")
        imageToTransmit = bridgeObject.cv2_to_imgmsg(capturedFrame, encoding="bgr8")
        publisher.publish(imageToTransmit)

        if capture_photo:
            rospy.loginfo("Capturing photo...")
            timestamp = time.strftime("%Y%m%d-%H%M%S")
            filename = os.path.join(photo_path, f"photo_{timestamp}.jpg")
            cv2.imwrite(filename, capturedFrame)
            rospy.loginfo(f"Photo saved to {filename}")
            capture_photo = False

    rate.sleep()
