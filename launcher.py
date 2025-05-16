import subprocess
import rospy
import time
import roslaunch
import signal
import sys
from mavros_msgs.msg import State

# Set a timeout for checking MAVROS heartbeat
HEARTBEAT_TIMEOUT = 10  # seconds

# Global launch objects for shutdown handling
mavros_launch = None
# tracker_launch = None
roscore_process = None


def check_roscore_running():
    """Check if roscore is running by trying to list ROS nodes."""
    try:
        subprocess.check_output("rosnode list", shell=True)
        return True
    except subprocess.CalledProcessError:
        return False


def start_roscore():
    """Start roscore using subprocess."""
    global roscore_process
    rospy.loginfo("Starting roscore...")
    roscore_process = subprocess.Popen(["roscore"])
    time.sleep(5)  # Wait for roscore to initialize properly


def launch_mavros(uuid):
    """Launch MAVROS node."""
    rospy.init_node("Mavros_Master_Master", anonymous=True)
    roslaunch.configure_logging(uuid)

    launch = roslaunch.parent.ROSLaunchParent(
        uuid,
        [
            "/home/astra/dynamic_flight/catkin_ws/src/waypoint_mavros/src/launch/roslaunch_mavros.launch"
        ],
    )
    launch.start()
    rospy.loginfo("MAVROS launched!")
    return launch


# def launch_tracker(uuid):
#     """Launch tracker.launch node."""
#     roslaunch.configure_logging(uuid)

#     launch = roslaunch.parent.ROSLaunchParent(
#         uuid,
#         ["/home/suas/detection_pipeline/detection_pipeline/catkin_ws/src/ultralytics_ros/launch/tracker.launch"]
#     )
#     launch.start()
#     rospy.loginfo("tracker.launch started!")
#     return launch


def check_mavros_heartbeat(timeout=HEARTBEAT_TIMEOUT):
    """Check if MAVROS is publishing heartbeats (communication with the vehicle)."""
    try:
        mavros_state = rospy.wait_for_message("/mavros/state", State, timeout=timeout)
        if mavros_state.connected:
            rospy.loginfo("Heartbeat received from MAVROS!")
            return True
        else:
            rospy.logwarn("MAVROS is not connected. Retrying heartbeat check.")
            return False
    except rospy.ROSException:
        rospy.logwarn(f"No heartbeat from MAVROS. Timeout: {timeout} seconds.")
        return False


def handle_shutdown(signum, frame):
    """Handle graceful shutdown on keyboard interrupt or termination."""
    rospy.loginfo("Shutting down due to interrupt or termination.")
    global roscore_process
    try:
        if mavros_launch:
            mavros_launch.shutdown()
        # if tracker_launch:
        #     tracker_launch.shutdown()
        if roscore_process:
            rospy.loginfo("Terminating roscore...")
            roscore_process.terminate()
            roscore_process.wait()  # Wait for it to actually exit
    except Exception as e:
        rospy.logwarn(f"Exception during shutdown: {e}")
    sys.exit(0)


if __name__ == "__main__":
    # Register signal handler for graceful shutdown on Ctrl+C (KeyboardInterrupt)
    signal.signal(signal.SIGINT, handle_shutdown)

    try:
        # Ensure roscore is running
        if not check_roscore_running():
            start_roscore()

        # Generate UUID
        uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)

        # Launch MAVROS
        mavros_launch = launch_mavros(uuid)

        # Wait for heartbeat
        while not rospy.is_shutdown():
            if not check_mavros_heartbeat():
                rospy.logwarn("No heartbeat from MAVROS. Restarting MAVROS launch...")
                mavros_launch.shutdown()
                mavros_launch = launch_mavros(uuid)
            else:
                rospy.loginfo("Heartbeat consistent. Turning off checks.")

                # Launch tracker.launch once heartbeat confirmed
                # tracker_launch = launch_tracker(uuid)
                break

            time.sleep(5)  # Check heartbeat every 5 seconds

        # Launch additional nodes
        gps_node = roslaunch.core.Node("gps_mavros", "gps.py")
        waypoint_node = roslaunch.core.Node("waypoint_mavros", "waypoint.py")
        image_pub = roslaunch.core.Node("video_cam", "image_pub.py")
        track_node = roslaunch.core.Node("ultralytics_ros", "track_node.py")
        result_sub = roslaunch.core.Node("ultralytics_ros", "yolo_result_sub.py")
        image_mapping = roslaunch.core.Node("image_mapping", "mapping.py")
        kill_node = roslaunch.core.Node("waypoint_mavros", "killnode.py")

        launch = roslaunch.scriptapi.ROSLaunch()
        launch.start()

        process = launch.launch(gps_node)
        process2 = launch.launch(waypoint_node)
        process3 = launch.launch(image_pub)
        process4 = launch.launch(track_node)
        process5 = launch.launch(result_sub)
        process6 = launch.launch(image_mapping)
        process7 = launch.launch(kill_node)

        rospy.spin()

    except Exception as e:
        rospy.logerr(f"An error occurred: {str(e)}")
        handle_shutdown(None, None)
