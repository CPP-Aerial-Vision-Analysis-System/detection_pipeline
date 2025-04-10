#!/usr/bin/env python3

import rospy
import time
from mavros_msgs.srv import WaypointPull, WaypointPush, WaypointClear, WaypointPushRequest
from mavros_msgs.msg import WaypointList, Waypoint, CommandCode, WaypointReached, State
from waypoint_mavros.srv import AddWaypoint, AddWaypointResponse, AddWaypointRequest

class WaypointManager:
    def __init__(self):
        rospy.init_node('waypoint_manager', anonymous=True)
        
        # Initialize connection state to False
        self.connected = False
        
        # Subscribers for state, waypoints, and waypoint reached notifications
        rospy.Subscriber("/mavros/state", State, self.state_callback)
        rospy.Subscriber("mavros/mission/waypoints", WaypointList, self.waypoints_callback)
        rospy.Subscriber('/mavros/mission/reached', WaypointReached, self.wp_reached_cb)
        
        # Service clients for waypoint operations
        self.waypoint_pull_client = rospy.ServiceProxy('/mavros/mission/pull', WaypointPull)
        self.waypoint_push_client = rospy.ServiceProxy('/mavros/mission/push', WaypointPush)
        self.waypoint_clear_client = rospy.ServiceProxy('/mavros/mission/clear', WaypointClear)
        
        # Initialize an empty waypoint list
        self.waypoint_list = WaypointList()

        self.waypoint_reached = 0

        self.service = rospy.Service("/AddWaypoint", AddWaypoint, self.handle_drone_waypoint_request)
        rospy.loginfo("Waypoint service ready")

    def state_callback(self, msg):
        """Callback to update the connection status from MAVROS."""
        self.connected = msg.connected

    def waypoints_callback(self, data):
        """Callback to store and log the received waypoints."""
        self.waypoint_list = data
        for i, wp in enumerate(data.waypoints):
            rospy.loginfo(f"Waypoint {i}: Lat:{wp.x_lat}, Long:{wp.y_long}, Alt:{wp.z_alt}")

    def push_waypoints(self):
        """Push the updated waypoint list to the flight controller."""
        rospy.wait_for_service('/mavros/mission/push')
        try:
            waypoint_push_request = WaypointPushRequest()
            waypoint_push_request.start_index = 0
            waypoint_push_request.waypoints = self.waypoint_list.waypoints
            push_result = self.waypoint_push_client(waypoint_push_request)
            rospy.loginfo(push_result)
        except rospy.ServiceException as e:
            rospy.loginfo(f"Service call failed: {e}")

    def clear_waypoints(self):
        """Clear the current waypoint list on the flight controller."""
        rospy.wait_for_service('/mavros/mission/clear')
        try:
            response = self.waypoint_clear_client()
            rospy.loginfo(f'Waypoint clear success: {response.success}')
        except rospy.ServiceException as e:
            rospy.loginfo(f"Service call failed: {e}")

    def insert_new_waypoint(self, lat, lon, alt, index):
        """Insert a new waypoint into the waypoint list and push the update."""
        new_waypoint = Waypoint()
        new_waypoint.frame = 3  # Global relative altitude
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
        rospy.loginfo(f"Inserting new waypoint at index {index}")
        self.waypoint_list.waypoints.insert(index, new_waypoint)
        self.push_waypoints()
        rospy.loginfo("New waypoint inserted into the WaypointList object.")

    def wp_reached_cb(self, msg):
        """Callback for when a waypoint is reached."""
        rospy.loginfo(f"Waypoint {msg.wp_seq} reached")
        self.waypoint_reached = msg.wp_seq

    def handle_drone_waypoint_request(self,req):
        response = AddWaypointResponse()
        self.insert_new_waypoint(req.latitude, req.longitude, req.altitude, index= self.waypoint_reached + 1)
        response.success = True
        return response
    
    def main(self):
        """Main function that waits for a confirmed connection before performing waypoint operations."""
        rospy.loginfo("Waiting for connection to be established...")
        while not rospy.is_shutdown() and not self.connected:
            rospy.sleep(0.5)
        rospy.loginfo("Connection established. Proceeding with waypoint operations.")

        # Wait for the waypoint pull service to be available and pull current waypoints
        rospy.wait_for_service('/mavros/mission/pull')
        try:
            response = self.waypoint_pull_client()
            if response.success:
                rospy.loginfo(f"Successfully received {response.wp_received} waypoints.")
            else:
                rospy.loginfo("Failed to pull waypoints.")
        except rospy.ServiceException as e:
            rospy.loginfo(f"Service call failed: {e}")

        # Example: Insert a new waypoint once connected and after pulling current waypoints
        # time.sleep(2)
        #self.insert_new_waypoint(34.0593371, -117.8211969, 101, int(response.wp_received))
        # time.sleep(2)
        
        rospy.spin()

if __name__ == '__main__':
    manager = WaypointManager()
    
    manager.main()
