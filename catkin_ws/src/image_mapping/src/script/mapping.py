#!/usr/bin/env python3

# import rospy
# from mavros_msgs.msg import WaypointReached, WaypointList
# import subprocess

# class MissionCameraTrigger:
#     def __init__(self):
#         rospy.init_node('mission_camera_trigger')
#         self.camera_trigger_indices = set()
#         self.current_mission = []
#         self.last_reached_seq = None

#         rospy.Subscriber('/mavros/mission/waypoints', WaypointList, self.waypoints_callback)
#         rospy.Subscriber('/mavros/mission/reached', WaypointReached, self.reached_callback)

#     def waypoints_callback(self, msg):
#         self.current_mission = msg.waypoints
#         rospy.loginfo_once(self.current_mission)
#         for idx, wp in enumerate(msg.waypoints):
#             if wp.command == 203:  # MAV_CMD_DO_DIGICAM_CONTROL
#                 self.camera_trigger_indices.add(idx)
#         rospy.loginfo_once(self.camera_trigger_indices)

#     # def reached_callback(self, msg):
#     #     seq = msg.wp_seq
#     #     if seq in self.camera_trigger_indices:
#     #         rospy.loginfo(f"Reached camera trigger waypoint {seq}, capturing image.")
#     #         self.trigger_camera()

#     def reached_callback(self, msg):
#         current_seq = msg.wp_seq
#         if self.last_reached_seq is not None:
#             skipped = range(self.last_reached_seq + 1, current_seq)
#             for skipped_seq in skipped:
#                 if skipped_seq in self.camera_trigger_indices:
#                     rospy.loginfo(f"Skipped to waypoint {current_seq}, triggering camera for skipped mission item {skipped_seq}")
#                     self.trigger_camera()

#         # Also check if current one is a camera trigger
#         if current_seq in self.camera_trigger_indices:
#             rospy.loginfo(f"Reached camera trigger waypoint {current_seq}, capturing image.")
#             self.trigger_camera()

#         self.last_reached_seq = current_seq

#     def trigger_camera(self):
#         # Replace with actual camera trigger logic
#         rospy.logwarn("Camera triggered. No camera attached")
from mavros_msgs.msg import StatusText
import rospy
import re


class MissionCameraTrigger:
    def __init__(self):
        rospy.init_node("mission_camera_trigger")
        rospy.Subscriber(
            "/mavros/statustext/recv", StatusText, self.statustext_callback
        )

    def statustext_callback(self, msg):
        if "DigiCamCtrl" in msg.text:
            match = re.search(r"Mission: (\d+)", msg.text)
            wp = match.group(1) if match else "?"
            rospy.loginfo(f"Camera trigger from DigiCamCtrl at waypoint {wp}")
            self.trigger_camera()

    def trigger_camera(self):
        rospy.loginfo("Triggering Jetson-side camera")
        # TODO: put camera logic here. (maybe a publisher to image_pub?)


if __name__ == "__main__":
    mapping_node = MissionCameraTrigger()
    rospy.spin()
