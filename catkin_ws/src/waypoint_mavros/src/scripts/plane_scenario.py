#!/usr/bin/env python3

import rospy
from mavros_msgs.msg import WaypointReached, State, StatusText
from mavros_msgs.srv import (
    WaypointSetCurrent,
    WaypointSetCurrentRequest,
    SetMode,
    CommandBool,
    CommandTOL,
)

ALT = 16.8


class Plane:
    def __init__(self):
        rospy.init_node("plane_scenario_node", anonymous=True)
        rospy.loginfo("Plane node initialized and listening for resume command.")

        rospy.Subscriber("/mavros/mission/reached", WaypointReached, self.wp_reached_cb)
        rospy.Subscriber(
            "/mavros/statustext/recv", StatusText, self.statustext_callback
        )

        self.message_sender = rospy.Publisher(
            "/mavros/statustext/send", StatusText, queue_size=10
        )

        rospy.wait_for_service("/mavros/set_mode")
        rospy.wait_for_service("/mavros/cmd/arming")
        rospy.wait_for_service("/mavros/mission/set_current")

        self.set_mode = rospy.ServiceProxy("/mavros/set_mode", SetMode)
        self.arm_srv = rospy.ServiceProxy("/mavros/cmd/arming", CommandBool)
        self.set_wp_srv = rospy.ServiceProxy(
            "/mavros/mission/set_current", WaypointSetCurrent
        )

        self.saved_waypoint = 0
        self.waypoint_reached = 0

    def wp_reached_cb(self, msg):
        """Callback for when a waypoint is reached."""
        self.waypoint_reached = msg.wp_seq
        rospy.loginfo(f"Reached waypoint {self.waypoint_reached}")

    def statustext_callback(self, msg):
        if "resume" in msg.text.lower():
            rospy.logwarn("Resume command received from GCS.")
            self.send_ack("Resume command received.")
            self.saved_waypoint = self.waypoint_reached + 1  # Resume from next waypoint
            self.resume_mission()

    def send_ack(self, text):
        """Send feedback to GCS (Mission Planner's Messages tab)."""
        status_msg = StatusText()
        status_msg.severity = 6  # INFO level. 6 = NOTICE
        status_msg.text = text
        self.message_sender.publish(status_msg)

    def change_mode(self, mode):
        rospy.loginfo(f"Setting mode to {mode}...")
        response = self.set_mode(custom_mode=mode)

        if response.mode_sent:
            rospy.loginfo(f"Mode changed to {mode}")
        else:
            rospy.logerr("Failed to change mode")

    def arm(self):
        rospy.loginfo("Arming drone...")
        response = self.arm_srv(True)
        if response.success:
            rospy.loginfo("Drone armed successfully.")
        else:
            rospy.logerr("Failed to arm drone.")

    def set_mission_index(self, wp_index):
        rospy.loginfo(f"Setting current mission waypoint to {wp_index}")
        req = WaypointSetCurrentRequest()
        req.wp_seq = wp_index
        response = self.set_wp_srv(req)
        if response.success:
            rospy.loginfo(f"Mission set to waypoint {wp_index}")
        else:
            rospy.logerr("Failed to set mission index.")

    def manual_takeoff(self, altitude):
        rospy.wait_for_service("/mavros/cmd/takeoff")
        takeoff = rospy.ServiceProxy("/mavros/cmd/takeoff", CommandTOL)
        resp = takeoff(0, 0, 0, 0, altitude)
        if resp.success:
            rospy.loginfo(f"Drone taking off to {altitude}m")
        else:
            rospy.logerr("Failed to initiate takeoff")

    def resume_mission(self):
        """Resume mission from the saved waypoint."""
        self.change_mode("GUIDED")
        rospy.sleep(2)
        self.arm()
        rospy.sleep(2)
        self.manual_takeoff(ALT)
        rospy.sleep(5)
        self.set_mission_index(self.saved_waypoint)
        self.change_mode("AUTO")
        self.send_ack(f"Resuming mission from waypoint {self.saved_waypoint}")


if __name__ == "__main__":
    try:
        plane = Plane()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
