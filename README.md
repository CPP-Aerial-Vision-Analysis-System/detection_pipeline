# This repo contains the object-detection flight software that would be used during the competition

## Table of Contents
- [Installations](#installations)
    - [Jetson](#jetson)
    - [Personal](#personal)
- [Dev]()
   - [Simulation](#simulation-setup)
   - [Running the pipeline](#running-the-pipeline)
- [TroubleShooting](#troubleshooting)
   - [Running Legacy Mavlink Code (most likely ignore)](#running-old-mavlink-code-ignore-most-likely)
   - [Jetson libgomp error fix](#if-you-get-libgomp-error-on-jetson)

---
## Installations
### Jetson (ubuntu 20.04)

**Steps:**

1. Git clone with submodules:
```bash
git clone --recurse-submodules https://github.com/CPP-Aerial-Vision-Analysis-System/detection_pipeline.git
```
2. `cd catkin_ws/src`  
3. `python3 -m pip install -r ultralytics_ros/requirements.txt`  
   - if on jetson make sure to install the torch and torchvision version compatible with jetpack. instructions here: [PyTorch for Jetson - NVIDIA Developer Forums](https://forums.developer.nvidia.com/t/pytorch-for-jetson/72048)  
4. `cd ..` (you should be in `catkin_ws` after this)  
5. `rosdep install -r -y -i --from-paths .`  
6. `catkin_make`  

### Personal

**Steps:**
1. Git clone with submodules:

```bash
git clone --recurse-submodules https://github.com/CPP-Aerial-Vision-Analysis-System/detection_pipeline.git
```

- *side note:* because of docker, every package comes preinstalled. kinda janky setup for how we actually made the docker image but it is what it is

2. Reopen in Devcontainer  
- if prompted, you can press *open* when it asks you if you want to open the devcontainer.  
- if you miss it or something, open the command pallete using `ctrl + shift + p` and press *Reopen in Container*.  
- **mount config error:** if you get this error, just press retry once. should fix itself.

## Simulation Setup

To run the simulation component of the project on your personal computer:

1. Make sure your system meets the requirements mentioned in the **Personal Computer Dev** section.
2. Once inside the devcontainer:
   - Launch the simulation environment using:
     ```bash
     roslaunch sitl-gazebo runway.launch
     ```
3. (Optional) Modify simulation parameters (e.g., camera feed, object positions, environment) in the corresponding launch or config files.
4. Start sitl interface with mavproxy:
   ```bash
   bash catkin_ws/src/sitl-gazebo/scripts/startsitl.sh
   ```
5. Start MAVROS:
```bash
roslaunch sitl-gazebo apm.launch
```
6. You can also run object detection within the sim by launching the pipeline as described in the next section but without launching mavros since it is already launched. **If doing this, make sure** that `input_topic` is set to `/webcam/image-raw/` instead of just `image_raw` when running simulation.

## Running the pipeline:
1. **(only on jetson)(optional)**. Run `sudo jetson_clocks`. This ensures that the jetson can be overclocked and run as efficiently as possible (run once per device boot).  
2. Open two terminals.  
3. In one terminal, `cd` into the `catkin_ws`  
4. Plug in Pixhawk (necessary) and GPS (optional), then:  
   - 4a. Run:
     ```bash
     roslaunch mavros px4.launch fcu_url:=<path to device connection>
     ```
     Make sure a heartbeat is detected.  
   - 4b. In the terminal with the `catkin_ws`, run:
     ```bash
     roslaunch ultralytics_ros tracker.launch
     ```
     Add `debug:=true` to the end if visual output (and not just text) is needed (off by default).  

---

### Running old mavlink code (ignore most likely)
- before following the "running the pipeline" instructions, copy the code from the old mavlink py file into `catkin_ws/src/ultralytics_ros/script/tracker_node.py`
- then follow instructions but skip 4c. 

### If you get libgomp error on jetson:
- run:
```bash
export LD_PRELOAD=/usr/lib/aarch64-linux-gnu/libgomp.so.1
```

---