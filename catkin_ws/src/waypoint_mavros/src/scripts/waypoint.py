#!/usr/bin/env python3

import rospy
import time
from mavros_msgs.srv import WaypointPull, WaypointPush, WaypointClear, WaypointPushRequest
from mavros_msgs.msg import WaypointList, Waypoint,CommandCode, WaypointReached

class WaypointManager:
    def __init__(self):
        rospy.init_node('waypoint_manager', anonymous=True)
        self.waypoint_list = WaypointList()
        self.waypoints_detection_list = []
        self.waypoint_pull_client = rospy.ServiceProxy('/mavros/mission/pull', WaypointPull)
        self.waypoint_push_client = rospy.ServiceProxy('/mavros/mission/push', WaypointPush)
        self.waypoint_clear_client = rospy.ServiceProxy('/mavros/mission/clear', WaypointClear)
        rospy.Subscriber("mavros/mission/waypoints", WaypointList, self.waypoints_callback)
        rospy.Subscriber('/mavros/mission/reached', WaypointReached, self.waypoint_reached)

    def waypoints_callback(self, data):
        self.waypoint_list = data
   
        for i, wp in enumerate(data.waypoints):
            new_wp = WaypointInfo(wp, "", {})
            self.waypoints_detection_list.append(new_wp)
            new_wp.__str__()
            rospy.loginfo(f"Waypoint {i}: Lat:{wp.x_lat}, Long:{wp.y_long}, Alt:{wp.z_alt}")

    def push_waypoints(self):
        rospy.wait_for_service('/mavros/mission/push')
        try:
            push_client = rospy.ServiceProxy('mavros/mission/push', WaypointPush)
            waypoint_push_request = WaypointPushRequest()
            waypoint_push_request.start_index = 0
            waypoint_push_request.waypoints = self.waypoint_list.waypoints
            push_result = push_client(waypoint_push_request)
            rospy.loginfo(push_result)
        except rospy.ServiceException as e:
            rospy.loginfo(f"Service call failed: {e}")

    def clear_waypoints(self):
        rospy.wait_for_service('/mavros/mission/clear')
        try:
            response = self.waypoint_clear_client()
            rospy.loginfo(f'Waypoint clear success: {response.success}')
        except rospy.ServiceException as e:
            rospy.loginfo(f"Service call failed: {e}")

    def insert_new_waypoint(self, lat, lon, alt, index):
        new_waypoint = Waypoint()
        new_waypoint.frame = 0  # Global relative altitude
        new_waypoint.command = 16  # MAV_CMD_NAV_WAYPOINT
        new_waypoint.is_current = False
        new_waypoint.autocontinue = True
        new_waypoint.param1 = 0
        new_waypoint.param2 = 0
        new_waypoint.param3 = 0
        new_waypoint.param4 = float("nan")
        new_waypoint.x_lat = lat
        new_waypoint.y_long = lon
        new_waypoint.z_alt = alt
        print(type(self.waypoint_list.waypoints))
        self.waypoint_list.waypoints.insert(index, new_waypoint)
        self.push_waypoints()

        rospy.loginfo("New waypoint inserted into the WaypointList object.")


    def remove_waypoint(self, index):
        self.waypoint_list.waypoints.pop(index)
        self.push_waypoints()
        rospy.loginfo("New waypoint REMOVED from the WaypointList object.")

    def create_waypoint(self):
        waypoint_list = WaypointPush()
        latitude = 50
        longitude = 10

        waypoint = Waypoint()
        waypoint.x_lat = latitude
        waypoint.y_long = longitude
        waypoint.autocontinue = True
        waypoint.is_current = False
        waypoint.frame = Waypoint.FRAME_GLOBAL

        waypoint.command = CommandCode.NAV_WAYPOINT
                     
        waypoint_list._request_class.waypoints.push_back(waypoint)
    
    def waypoint_reached(msg):
        global crossed_waypoints
        rospy.loginfo(f"Waypoint {msg.wp_seq} reached")
        crossed_waypoints.append(msg.wp_seq)

    def main(self):
        rospy.wait_for_service('/mavros/mission/pull')
        try:
            response = self.waypoint_pull_client()
            # response.
            if response.success:
                rospy.loginfo(f"Successfully received {response.wp_received} waypoints.")
            else:
                rospy.loginfo("Failed to pull waypoints.")
        except rospy.ServiceException as e:
            rospy.loginfo(f"Service call failed: {e}")

        # Example of inserting a new waypoint
        #self.insert_new_waypoint(10.7749, -12.4194, 90, 2)
        time.sleep(2)
        # self.insert_new_waypoint(2, 2, 2, 3)
        # time.sleep(10)
        #self.remove_waypoint(0)
        # self.create_waypoint()
        
        rospy.spin()

class WaypointInfo:
    def __init__(self, waypoint, object_detected, calc_obj_waypoint):
        self.waypoint = waypoint
        self.object_detected = object_detected
        self.calc_obj_waypoint = calc_obj_waypoint

    def __str__(self):
        return f"Waypoint = 
            Lat:{self.waypoint.x_lat}, Long:{self.waypoint.y_long}, Alt:{self.waypoint.z_alt} 
            Object Detected = {self.object_detected} 
            Calc Waypoint = 
            Lat:{self.calc_obj_waypoint.x_lat}, Long:{self.calc_obj_waypoint.y_long}, Alt:{self.calc_obj_waypoint.z_alt} "


if __name__ == '__main__':
    manager = WaypointManager()
    manager.main()