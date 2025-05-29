#!/bin/bash

# Configs
MAVROS_CMD="roslaunch waypoint_mavros roslaunch_mavros.launch"
TRACKER_CMD="roslaunch ultralytics_ros tracker.launch"
MAX_WAIT=30
LOOP_DELAY=1

# Source ROS environment
# source /opt/ros/noetic/setup.bash
# cd ~/detection_pipeline/catkin_ws
# source devel/setup.bash

# Start MAVROS
echo "[INFO] Launching MAVROS..."
$MAVROS_CMD &
MAVROS_PID=$!

# Wait for heartbeat
echo "[INFO] Waiting for heartbeat on /mavros/state..."
HEARTBEAT=false
for (( i=0; i<$MAX_WAIT; i++ )); do
    if rostopic echo -n 1 /mavros/state > /dev/null 2>&1; then
        echo "[INFO] Heartbeat detected."
        HEARTBEAT=true
        break
    fi
    sleep $LOOP_DELAY
done

# Launch tracker if heartbeat received
if $HEARTBEAT; then
    echo "[INFO] Launching Ultralytics Tracker..."
    $TRACKER_CMD
else
    echo "[ERROR] No heartbeat received after $MAX_WAIT seconds."
fi

# Keep MAVROS alive
wait $MAVROS_PID
