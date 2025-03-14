#!/usr/bin/env python3

import rospy
from mavros_msgs.msg import State
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Float64, Bool
from mavros_msgs.srv import StreamRate
from gps_mavros.srv import GetGPSData, GetGPSDataResponse
import threading

# Global variables
connected = False
latest_gps = None
latest_yaw = None


def state_cb(msg):
    """Callback to monitor MAVROS connection state."""
    global connected
    connected = msg.connected

def gps_cb(msg):
    """Callback to store the latest GPS data."""
    global latest_gps
    latest_gps = msg

def pose_callback(msg):
    """Callback to store the latest yaw data (Z-axis rotation)."""
    global latest_yaw
    latest_yaw = msg.data

def set_stream_rate(stream_id=0, message_rate=10, on_off=True):
    """Set MAVROS data stream rate."""
    rospy.wait_for_service('/mavros/set_stream_rate')
    try:
        set_rate = rospy.ServiceProxy('/mavros/set_stream_rate', StreamRate)
        set_rate(stream_id, message_rate, on_off)
        rospy.loginfo(f"Stream rate set: ID={stream_id}, Rate={message_rate}Hz")
    except rospy.ServiceException as e:
        rospy.logerr(f"Failed to set stream rate: {e}")

def handle_drone_data_request(req):
    """Service callback to return the latest GPS and yaw data."""
    if latest_gps and latest_yaw is not None:
        response = GetGPSDataResponse()
        response.latitude = latest_gps.latitude
        response.longitude = latest_gps.longitude
        response.altitude = latest_gps.altitude
        response.yaw = latest_yaw
        return response
    else:
        rospy.logwarn("No GPS or yaw data received yet!")
        return GetGPSDataResponse(0, 0, 0, 0)

def heartbeat():
    """Heartbeat function to monitor connection status only on change."""
    rate = rospy.Rate(1)  # 1 Hz
    last_connected = None
    while not rospy.is_shutdown():
        if connected != last_connected:
            if connected:
                rospy.loginfo("Heartbeat: Connected to Pixhawk")
            else:
                rospy.logwarn("Heartbeat: Disconnected from Pixhawk")
            last_connected = connected
        rate.sleep()



if __name__ == "__main__":
    rospy.init_node("gps_mavros_service_node", anonymous=True)
    
    # Set MAVROS stream rate
    set_stream_rate(stream_id=0, message_rate=1, on_off=True)

    # Subscribers
    rospy.Subscriber("/mavros/state", State, state_cb)
    rospy.Subscriber("/mavros/global_position/global", NavSatFix, gps_cb)
    rospy.Subscriber("/mavros/global_position/compass_hdg", Float64, pose_callback)

    # Define ROS Service
    service = rospy.Service("/get_drone_data", GetGPSData, handle_drone_data_request)
    rospy.loginfo("Drone Data Service Ready.")

    # Heartbeat thread
    heartbeat_thread = threading.Thread(target=heartbeat)
    heartbeat_thread.daemon = True
    heartbeat_thread.start()


    rospy.spin()