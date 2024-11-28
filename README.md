# This repo contains the object-detection flight software that would be used during the competition

## Running old mavlink code
- before following the "running the pipeline" instructions, copy the code from the old mavlink py file into `catkin_ws/src/ultralytics_ros/script
/tracker_node.py`
- then follow instructions but skip 4c. 

## Dev

## Running the pipeline:
1. run `systemctl stop docker`. this stops docker to free ram (ran once per device boot)
2. run `sudo jetson_clocks`. this ensures that the jetson can be overclocked and run as efficient as possible (ran once per device boot)
3. cd into the catkin_ws
4. open 3 terminals and `source devel/setup.bash` in all of them. also make sure theyre all in the catkin workspace
- 4a. terminal1: `roslaunch ultralytics_ros tracker.launch` (add `debug:=true` after the tracker.launch if you want a visual output of the results)
- 4b. terminal2: `rosrun video_cam image_pub.py` (publishes camera frames)
- 4c. terminal3: `rosrun ultralytics_ros yolo_result_sub` (prints out detections textually)


### If you get libgomp error on jetson:
- run `export LD_PRELOAD=/usr/lib/aarch64-linux-gnu/libgomp.so.1`
