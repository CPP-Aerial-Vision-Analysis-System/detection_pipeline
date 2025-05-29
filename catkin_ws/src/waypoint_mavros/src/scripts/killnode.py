#!/usr/bin/env python

import rospy
import os
from mavros_msgs.msg import StatusText
import rosnode


class KillNode:
    def __init__(self):
        rospy.init_node("flight_kill_node", anonymous=True)
        rospy.Subscriber(
            "/mavros/statustext/recv", StatusText, self.statustext_callback
        )
        self.message_sender = rospy.Publisher(
            "/mavros/statustext/send", StatusText, queue_size=10
        )
        rospy.loginfo("KillNode initialized and listening for shutdown commands.")

    def statustext_callback(self, msg):
        if "shutdown" in msg.text.lower():  # Case-insensitive match
            rospy.logwarn("Jetson Shutdown Triggered. Shutting down...")
            self.send_ack("Shutdown command received.")
            self.shutdown_nodes()
            self.shutdown_jetson()

    def send_ack(self, text):
        # feedback to gcs (mission planner in messages tab)
        status_msg = StatusText()
        status_msg.severity = 6  # INFO level. 6=notice
        status_msg.text = text
        self.message_sender.publish(status_msg)

    def shutdown_nodes(self):
        # Optional: shutdown other ROS nodes gracefully
        rospy.loginfo("Shutting down other ROS nodes...")
        current_node = rospy.get_name()
        nodes_list = []
        for node in rosnode.get_node_names():
            if node != current_node:
                nodes_list.append(node)

        try:
            rosnode.kill_nodes(nodes_list)
            rospy.loginfo(f"Killed nodes: {nodes_list}")
        except Exception as e:
            rospy.logwarn(f"Failed to kill node {node}: {e}")

    def shutdown_jetson(self):
        rospy.loginfo("Executing Jetson shutdown command...")
        password = "UAV_Lab"
        os.system(f"echo {password} | sudo -S shutdown now")


if __name__ == "__main__":
    try:
        mapping_node = KillNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
