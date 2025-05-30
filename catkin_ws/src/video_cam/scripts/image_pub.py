#!/usr/bin/env python3

import rospy
import cv2
from sensor_msgs.msg import Image
from std_msgs.msg import Bool, Float64
from cv_bridge import CvBridge
import os, time
import rospkg
from mavros_msgs.msg import StatusText

publisherNodeName = "camera_sensor_publisher"
topicName = "image_raw"
rp = rospkg.RosPack()
photo_path = os.path.join(rp.get_path("video_cam"), "camera_feed")
mapping_photo_path = os.path.join(rp.get_path("video_cam"), "mapping")


if not os.path.exists(photo_path):
    os.makedirs(photo_path)

if not os.path.exists(mapping_photo_path):
    os.makedirs(mapping_photo_path)

rospy.init_node(publisherNodeName, anonymous=True)
publisher = rospy.Publisher(topicName, Image, queue_size=1)
status_publisher = rospy.Publisher("/mavros/statustext/send", StatusText, queue_size=10)
rate = rospy.Rate(30)

#flag to indicate if we are using real camera or not
use_real_camera = rospy.get_param("~real_camera", False)
rospy.loginfo(f"Using real camera: {use_real_camera}")

bridgeObject = CvBridge()
# flag to indicate when to save photo to disk
capture_photo = False
# flag to indicate if camera is enabled/disabled
camera_enabled = False
ALT_THRESHOLD = 13.716

#Camera setup
latest_image_msg = None
if use_real_camera:
    pipeline = (
    "nvarguscamerasrc ! "
    "video/x-raw(memory:NVMM), width=1280, height=720, format=NV12, framerate=30/1 ! "
    "nvvidconv flip-method=0 ! "
    "video/x-raw, format=BGRx ! "
    "videoconvert ! "
    "video/x-raw, format=BGR ! appsink drop=true max-buffers=1 sync=false"
    )
    videoCaptureObject = cv2.VideoCapture(pipeline, cv2.CAP_GSTREAMER)
    rospy.loginfo("Using real camerea GStreamer")
else:
    videoCaptureObject = cv2.VideoCapture(0)

    def sim_image_callback(msg):
        global latest_image_msg
        latest_image_msg = msg
    
    rospy.Subscriber("/webcam/image_raw", Image, sim_image_callback)
    rospy.loginfo("Using simulation camera")
    
#Callbacks
def send_ack(text):
    # feedback to gcs (mission planner in messages tab)
    status_msg = StatusText()
    status_msg.severity = 6  # INFO level. 6=notice
    status_msg.text = text
    status_publisher.publish(status_msg)


def camera_trigger_callback(msg):
    global capture_photo
    if msg.data:
        rospy.loginfo("Camera trigger received. Preparing to capture a photo.")
        capture_photo = True


def check_altitude(msg):
    global camera_enabled
    current_alt = msg.data
    # rospy.loginfo(f"{current_alt}, {ALT_THRESHOLD}, {camera_enabled}")
    if current_alt >= ALT_THRESHOLD:
        if not camera_enabled:
            rospy.loginfo("Min Altitude reached. Enabling detection")
            send_ack("Min Altitude reached. Enabling detection")
        camera_enabled = True
    else:
        if camera_enabled:
            rospy.loginfo("Ideal Altitude not reached. Disabling detection")
            send_ack("Ideal Altitude not reached. Disabling detection")
        camera_enabled = False


rospy.Subscriber("/camera/trigger", Bool, camera_trigger_callback)
rospy.Subscriber("/mavros/global_position/rel_alt", Float64, check_altitude)


while not rospy.is_shutdown():
    if camera_enabled:
        if use_real_camera:
            returnValue, capturedFrame = videoCaptureObject.read()
            if returnValue == True:
                rospy.loginfo_once("Begun Camera Frame Publishing")
                imageToTransmit = bridgeObject.cv2_to_imgmsg(capturedFrame, encoding="bgr8")
                publisher.publish(imageToTransmit)
                timestamp = time.strftime("%Y%m%d-%H%M%S")
                filename = os.path.join(photo_path, f"photo_{timestamp}.jpg")
                cv2.imwrite(filename, capturedFrame)

                if capture_photo:
                    rospy.loginfo("Capturing photo...")
                    timestamp = time.strftime("%Y%m%d-%H%M%S")
                    filename = os.path.join(
                        mapping_photo_path, f"mapping_photo_{timestamp}.jpg"
                    )
                    cv2.imwrite(filename, capturedFrame)
                    rospy.loginfo(f"Photo saved to {filename}")
                    capture_photo = False
        else:
            if latest_image_msg is not None:
                rospy.loginfo_once("Begun Camera Frame Republishing")
                publisher.publish(latest_image_msg)

                # Save image to disk using cv_bridge
                cv_image = bridgeObject.imgmsg_to_cv2(latest_image_msg, desired_encoding="bgr8")
                timestamp = time.strftime("%Y%m%d-%H%M%S")
                filename = os.path.join(photo_path, f"photo_{timestamp}.jpg")
                cv2.imwrite(filename, cv_image)

                if capture_photo:
                    rospy.loginfo("Capturing photo...")
                    filename = os.path.join(
                        mapping_photo_path, f"mapping_photo_{timestamp}.jpg"
                    )
                    cv2.imwrite(filename, cv_image)
                    rospy.loginfo(f"Photo saved to {filename}")
                    capture_photo = False
                
    rate.sleep()