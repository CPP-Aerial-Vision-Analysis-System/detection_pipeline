# This repo contains the object-detection flight software that would be used during the competition

## Running old mavlink code
- before following the "running the pipeline" instructions, copy the code from the old mavlink py file into `catkin_ws/src/ultralytics_ros/script
/tracker_node.py`
- then follow instructions but skip 4c. 

## Dev (for redownloading on jetson or on any machine with ros noetic)
1. Git clone with submodules
- git clone --recurse-submodules https://github.com/CPP-Aerial-Vision-Analysis-System/detection_pipeline.git
2. `cd catkin_ws/src`
3. `python3 -m pip install -r ultralytics_ros/requirements.txt`
- if on jetson make sure to install the torch and torchvision version compatible with jetpack. instructions here: [PyTorch for Jetson - NVIDIA Developer Forums](https://forums.developer.nvidia.com/t/pytorch-for-jetson/72048)
4. `cd ..` (you should be in catkin_ws after this)
5. `rosdep install -r -y -i --from-paths .`
6. `catkin_make`

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
