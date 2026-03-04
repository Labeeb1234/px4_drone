# PX4 Drone - Autonomous Testing & Navigation Package

An open-source testing and development package for autonomous drone systems with advanced obstacle avoidance, path planning, and spatial understanding capabilities.

- currently undergoing both hardware (due to some unforseen outcomes in the previous tests) and software updates (porting to ROS2 and structuring the different coms architecture)
  <div>
   <img src="https://github.com/user-attachments/assets/097cda33-6b2d-4e2c-8537-0c06e108bddf" alt="Quadcoptor-Hardware" height="480" width="640"/>
  </div>


## Overview

This repository contains a complete autonomous drone system built on the PX4 autopilot stack. It integrates multiple planning and perception modules to enable safe autonomous navigation in complex environments.

**Hardware Configuration:**
- Platform: S500 Quadrotor Frame
- Total Weight: ~2.0 kg (including all computational boards)
- Companion Computer/computational boards: Raspi 4B 8GB RAM
- Development Environment for companion computer: ROS2 Humble -- Ubuntu 22.04 ARM64 Mate version(Jelly Jammy) 

**Software Configuration:**

- **OS**: Ubuntu 22.04
- **ROS**: ROS2 Humble
- **PX4 Autopilot**: Latest stable version
- **QGC**: QGroundControl (Mission planning GUI)
- **C++ Compiler**: C++11 or higher
- **Depth Camera**: RealSense D435(i)


## Hardware setup process

- Finished the sensor, radio, actuator, battery voltage, ESC, safety configuration and setups
- RC transmitter and receiver binding process (find online very common stuff) {out model: Flysky FS-i6X is a budget-friendly 10-channel 2.4GHz AFHDS 2A radio transmitter}
- Currently tuning the hardware on testing: (Disable MC_AIR_MODE on tuning to prevent take-off threshold from forcing too high of a throttle; renable after tuning though)
  - Pre-Tuning flight check (done flight is stable enough in RC-mode to be satisfied with auto-tuning of the main two controllers) 
  - PID tuning of rate and altitude controllers (auto tuning them in stabilize or altitude mode)  
  - Velocity and Position controller tuning (done manually in position mode mostly)
- Twisting paired signal wirings (ESC motor contoller signal wirings) to reduce magnetic interference (need to verify if this even works, someone??)
  
## Key Features

<!-- ### 🚁 Autonomous Navigation
- **3DVFH+* Local Planner**: Advanced obstacle avoidance and dynamic path replanning
- **Fuel Planner**: Fast trajectory generation for agile autonomous flight
- **Real-time Processing**: High-quality trajectory outputs within milliseconds
- **Unknown Environment Support**: Fully autonomous flight in cluttered, dynamic environments

### 🧠 Spatial Understanding
- **SpatialLM Integration**: Point cloud understanding using LLM models
- **Object Detection**: Advanced perception on processed point cloud data
- **Semantic Navigation**: Understand spatial layouts and relationships using natural language
- **RealSense D435(i) Support**: Integrated depth camera processing

### 📊 Perception & Planning Modules
- **Point Cloud Processing**: `pcd_saver` - Capture and save point cloud data
- **Exploration Management**: `fuel_planner/exploration_manager` - Autonomous exploration strategies
- **Local Planning**: `local_planner` - Real-time obstacle avoidance
- **Avoidance Systems**: `avoidance` - Advanced collision avoidance algorithms -->

## System Arch And Software Testing (TBC)

- Test system arch (high level)
  <div>
   <img src="https://github.com/user-attachments/assets/cdab23c8-f234-4a42-ae05-5ee896728bf7" alt="High-Level-Architecture" />
  </div>

- Currently testing different coms and connection archs for the drone FCU and raspi (companion computer): Mavlink(mavlink_c or mavsdk) serial coms mode, uXRCE-DDS
- Mavlink with serial coms works fine was able to read data properly, tested extracting the ATTITUDE data from the FCU
- Refer this [doc](https://docs.px4.io/main/en/companion_computer/pixhawk_rpi) for knowing the hardware and firmware setup to enable UART serial coms between raspi and pixhawk

- **Note**: make sure to use a working FCU 😭



### Required Dependencies

- Install Ubuntu Mate (version: 22.04), a mimimal Ubuntu desktop type OS optimized for raspi
- For Mavlink installation refer [this](https://mavlink.io/en/getting_started/installation.html); I used the C library (mavlink 2 version) for testing and understanding but there are versions of mavlink framwork for many other languages too check the doc. For mavlink2_c use this [doc](https://github.com/mavlink/c_uart_interface_example/blob/master/autopilot_interface.cpp#L596) to learning about the APIs
- Install ROS2-Humble (follow doc) same steps work for ARM64 systems too.

- **Note**: You do not need to install or generate the source files if you are using the C programming language and a standard dialect. Just get the prebuilt libraries and then jump to Using C Libraries. For custom dialects (message types written in xml format) you also need mavgen for generate custom message interface libs; ser [here](https://mavlink.io/en/mavgen_c/#get_libraries)

- **More coming as the testing progresses**


## Installation & Setup

### 3. PX4 Autopilot Setup
Follow the installation guide at [PX4 Avoidance](https://github.com/PX4/PX4-Avoidance) to set up the 3DVFH+* local planner.

### 4. QGroundControl Setup
Download and install [QGroundControl](http://qgroundcontrol.com/) for mission planning and drone configuration.

### 5. Realsense SDK+ROS2 Wrapper setup on Raspi-4B Model

- Kind of struggled with this due to really resource contraining while acessing both rgb,depth and point_cloud data from the realsense d435 device.
- The entire process is documented below:-

  #### Installation of Realsense pkg on Raspi 4b (Ubuntu 22.04, ROS2-Humble,Iron)

    - For rapsi 4b model the D435 pkg, both SDK and ROS2 wrapper has to be installed from source or for any ARM arch system installation on any AMD systems its fine to install via binaries or other method w/o any issues or conflicts in the firmware.
    - [Realsense SDK](https://github.com/realsenseai/librealsense)  (install latest version)
    - [ROS2 Wrapper](https://github.com/realsenseai/realsense-ros)  (install either latest version or the compatible one with the sdk version)
    - Follow this [link](https://github.com/Tianyuan-Wang/RealSense-ROS2-Raspberry-Pi-4) for the entire installation.
    - Even after building with flags on raspi 4b due to resource constraints with higher resolution and FPS of the RGB and depth frames it might show warnings like **resource unavailable** which can be reduced or prevent by using lower compatible resolution and FPS 
    - Make sure both depth and rgb streams have the same profile(resolution+FPS) for syncing and enabling pointcloud data.
    - Installation process on Raspi 4B:
	  - Check the realsense hardware firmware version (our case: d435-5.16.0.1)
	  - Install the compatible version of librealsense2 SDK **from source** based on the firmware's compatible SDK version ranges (our case: SDK-v2.55.1)
	  - Install the compatible version of the ROS2 realsense wrapper based on the ROS-DISTRO, linux OS version, realsense fm version & realsense-SDK version (our case: ROS2-Wrapper-v4.55.1)
    - Resource not available issue still persists, should I check OAK-D Lite on Raspi-4B to extract point cloud data with the DepthAI ros2 driver for stereo-cams.
  	- **With certain settings both RGB-frames and depth frames along with point cloud was extracted successfully though this is not always reliable, works only at lower res and FPS. (Higher FPS and res increases resource usage and power and leads to the warning mentioned below)**
  	- ```bash
    	ros2 launch realsense2_camera rs_launch.py enable_color:=true enable_depth:=true pointcloud.enable:=true rgb_camera.color_profile:=424x240x15 depth_module.depth_profile:=424x240x15 <or any   other valid profiles given below for both rgb and depth>
    	```
  	- Working depth and color frame profiles:
	  	- 424x240,6
	  	- 424x240,15
	  	- 640x480,6 (higher power consumption doesn't work with powerbank)

  	- **Note**: while building make sure to use the given build flags to prevent the node from getting interrupted due to errors , don't skip this flag **-DFORCE_RSUSB_BACKEND=true** 
  	- ```bash
  	  	- 29/04 15:48:04,815 ERROR [281473063839968] (librealsense-exception.h:52) get_xu(...). xioctl(UVCIOC_CTRL_QUERY) failed Last Error: No such file or directory
    	```
  	-  if the SDK was build with the required build flags under resource constrain the above msg will turn into a warning saying the below
  	- ```bash
	    - 15/02 15:14:43,719 WARNING [547764231968] (messenger-libusb.cpp:42) control_transfer returned error, index: 768, error: Resource temporarily unavailable, number: 11
    	```
  	  
## Testings And Experimentation

### Simulation

- A portion of a fork from 'PX4-Autopilot' repo is uploaded here in this repo for personal uses (may or may not be useful) [here]()
- For a quick sim testing and quick path planning test readiness, I directly bridged over the required gz topics and created the appropriate TF-broadcaster using the gz odom data using the ros_gz_bridge node
- Note don't do the above yet for proper testing of the ROS2-PX4 integration pipeline, which need XRCE-DDS (microROS) ---> do this first which make the replication process for the hardware easy (Done and verified this already working fine)

- **Required CMDs (as of now subject to changes later on for simplicity)**
```bash
export PX4_GZ_WORLD=open_world (this was created inside px4-autopilot pkg in the gz sim's world files named open_world.sdf)
```
```bash
make px4_sitl gz_x500_obs
```
```bash
ros2 launch sim_bridger gz_bridge.launch.py use_sim_time:=true (by default) (only launch for sim purposes only for bridging gz topics to ROS2)
```
```bash
ros2 run octomapping_test odom_tf_node (for odom->base_link tf)
```
```bash
ros2 launch octomapping_test octomapping.launch.py (for static base_link->cam_link tf and launching octomapping server for octomapping)
```


- **Note**: The static TFs are based on the dimensions of the X500 drone model dimensions specified in the sdf in the px4-autopilot pkg


### Hardware Mission Planning
1. Connect drone via USB or telemetry link to QGroundControl
2. Plan mission waypoints in QGC interface
3. Upload mission to drone
4. Arm and start autonomous flight

- The above structure to plan global paths for the drone navigation remains the same
- Faced an **issue: 'Terrain Following feature not working, when the height reference param(EKF2_HGT_REF) was set to the connected periferal distance sensor the GPS was alwasys locked preventing Mission Planning and navigation'** --> **reason found: Incompatiblility of autopilot version installed in the pixhawk-6C model FCU, we downgraded to 1.14.3 from 1.16.0(latest of 2026) and the issue was resolved**
- Both normal navigation and terrain following enabled navigation worked without any issues (no obstacle avoidance integrated yet)

**currently waiting to get new power supply to power the on-board raspi for running the avoidance algorithms and realsense RGB-D cam for proper navigation pipeline completion**

<!-- ## Usage

### Point Cloud Processing
```bash
# Capture and save point cloud data
rosrun pcd_saver pcd_saver_node

# Process with SpatialLM
roslaunch spatial_lm spatial_lm.launch
```

## Module Documentation

### Fuel Planner (Fast Planner)
High-performance trajectory planning system for aggressive autonomous flight: 
- Online mapping with depth images
- Kinodynamic path searching (A* algorithm variant)
- B-spline trajectory optimization
- Heading/yaw angle planning

See [fuel_planner/README.md](fuel_planner/exploration_manager/README.md) for detailed documentation.

### Local Planner
Real-time obstacle avoidance using 3DVFH+* algorithm:
- Dynamic reconfigurable parameters
- Depth sensor integration
- Velocity command generation
- Safety constraints enforcement

### Avoidance Module
Collision prevention system integrating with PX4 autopilot:
- Real-time threat detection
- Emergency maneuvering
- Failsafe protocols

### Point Cloud Saver
Data capture utility for offline analysis:
- RealSense D435(i) integration
- PCD format export
- Timestamped logging

## Spatial LM Integration

This project includes [SpatialLM](https://huggingface.co/manycore-research/SpatialLM-Llama-1B) - a point cloud understanding LLM model: 

**Capabilities:**
- Object detection on 3D point clouds
- Spatial relationship understanding
- Natural language spatial queries
- Scene interpretation for autonomous decision-making

## Video Demonstrations

Check out the navigation system in action:
- [3DVFH+* Local Planner Demo](https://github.com/Labeeb1234/px4_drone)

## Configuration

Key configuration files are located in `fuel_planner/plan_manage/config/`:

- `simulation.launch` - Simulation parameters
- `kino_algorithm.xml` - Kinodynamic planner settings
- `realsense.launch` - Depth camera configuration

Adjust these parameters based on your hardware and environment. 

## Troubleshooting

**Issue**: Drone not responding to commands
- Solution: Check USB connection, verify baud rate, restart autopilot

**Issue**: Poor obstacle detection
- Solution: Check camera calibration, ensure adequate lighting, verify depth sensor connection

**Issue**: Slow trajectory generation
- Solution: Reduce map resolution, enable GPU acceleration, optimize downsampling parameters


## References & Attribution

This project builds upon:
- [PX4 Avoidance](https://github.com/PX4/PX4-Avoidance) - 3DVFH+* implementation
- [Fast-Planner](https://github.com/HKUST-Aerial-Robotics/Fast-Planner) - Trajectory optimization
- [SpatialLM](https://huggingface.co/manycore-research/SpatialLM-Llama-1B) - Spatial understanding

--- -->

**Last Updated**: Feb 2026  
**Repository**: [Labeeb1234/px4_drone](https://github.com/Labeeb1234/px4_drone)
**Status**: Active Development
