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
2. run `sudo jetson_clocks`. this ensures that the jetson can be overclocked and run as efficient as possible (ran once per device boot)
3. open two terminals.
4. cd into the catkin_ws in one terminal
4a. plug in pixhawk (necessary) and gps (optional), and run `roslaunch mavros px4.launch fcu_url:=<path to device connection>`. Make sure a heartbeat is detected
4b. in the terminal with the catkin ws, run `roslaunch ultralytics_ros tracker.launch`. add `debug:=true` to the end if visual output (and not just text) is needed (off by default).


### If you get libgomp error on jetson:
- run `export LD_PRELOAD=/usr/lib/aarch64-linux-gnu/libgomp.so.1`
