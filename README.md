# PX4 Drone - Autonomous Testing & Navigation Package

An open-source testing and development package for autonomous drone systems with advanced obstacle avoidance, path planning, and spatial understanding capabilities.

## Overview

This repository contains a complete autonomous drone system built on the PX4 autopilot stack. It integrates multiple planning and perception modules to enable safe autonomous navigation in complex environments.

**Hardware Configuration:**
- Platform: S500 Quadrotor Frame
- Total Weight: ~2.0 kg (including all computational boards)
- Development Environment: ROS Noetic on Ubuntu 20.04 (Focal Fossa)

## Key Features

### 🚁 Autonomous Navigation
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
- **Avoidance Systems**: `avoidance` - Advanced collision avoidance algorithms

## System Architecture

```
px4_drone/
├── avoidance/              # Collision avoidance module
├── fuel_planner/           # Fast trajectory planning system
│   ├── exploration_manager/
│   └── plan_manage/
├── local_planner/          # Local navigation and obstacle avoidance
├── pcd_saver/              # Point cloud data capture
└── README.md
```

## Prerequisites

- **OS**: Ubuntu 20.04 (Focal Fossa)
- **ROS**: ROS Noetic
- **PX4 Autopilot**: Latest stable version
- **QGC**: QGroundControl (Mission planning GUI)
- **C++ Compiler**: C++11 or higher
- **Depth Camera**: RealSense D435(i) (optional but recommended)

### Required Dependencies
```bash
# Install ROS Noetic
sudo apt-get install ros-noetic-desktop-full

# Install PX4 dependencies
sudo apt-get install python3-pip python3-numpy
pip3 install --user pyserial pyulog

# Install additional tools
sudo apt-get install ros-noetic-realsense2-camera
sudo apt-get install ros-noetic-pointcloud-to-laserscan
```

## Installation & Setup

### 1. Clone the Repository
```bash
cd ~/catkin_ws/src
git clone https://github.com/Labeeb1234/px4_drone.git
cd ~/catkin_ws
```

### 2. Build the Workspace
```bash
catkin_make
source devel/setup.bash
```

### 3. PX4 Autopilot Setup
Follow the installation guide at [PX4 Avoidance](https://github.com/PX4/PX4-Avoidance) to set up the 3DVFH+* local planner.

### 4. QGroundControl Setup
Download and install [QGroundControl](http://qgroundcontrol.com/) for mission planning and drone configuration.

## Usage

### Launch the Navigation Stack
```bash
# Terminal 1: Start RViz visualization
roslaunch px4_drone visualization. launch

# Terminal 2: Start autonomous navigation system
roslaunch px4_drone navigation.launch

# Terminal 3: (Optional) Start point cloud saver
roslaunch pcd_saver pcd_saver.launch
```

### Mission Planning
1. Connect drone via USB or telemetry link to QGroundControl
2. Plan mission waypoints in QGC interface
3. Upload mission to drone
4. Arm and start autonomous flight

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

## Contributing

Contributions are welcome! Please: 
1. Fork the repository
2. Create a feature branch (`git checkout -b feature/AmazingFeature`)
3. Commit your changes (`git commit -m 'Add AmazingFeature'`)
4. Push to the branch (`git push origin feature/AmazingFeature`)
5. Open a Pull Request

## References & Attribution

This project builds upon:
- [PX4 Avoidance](https://github.com/PX4/PX4-Avoidance) - 3DVFH+* implementation
- [Fast-Planner](https://github.com/HKUST-Aerial-Robotics/Fast-Planner) - Trajectory optimization
- [SpatialLM](https://huggingface.co/manycore-research/SpatialLM-Llama-1B) - Spatial understanding

## License

This project is open source.  Please refer to individual module licenses: 
- Fast-Planner modules: GPLv3
- PX4 integration: BSD 3-Clause

## Disclaimer

This is a research and development project. Use at your own risk.  Autonomous flight systems can be dangerous. Always:
- Test in controlled environments
- Follow local aviation regulations
- Maintain line of sight during initial tests
- Use appropriate safety measures (crash nets, clear areas, etc.)

## Support & Contact

For issues, questions, or suggestions:
- Open an issue on GitHub
- Check existing issues and discussions
- Review module-specific documentation

---

**Last Updated**: December 2025  
**Repository**: [Labeeb1234/px4_drone](https://github.com/Labeeb1234/px4_drone)  
**Language**: C++  
**Status**: Active Development



<!-- ## Autonomous Drone
- Hardware used is : S500 Frame with components in a kit
- Weight of the entire drone with all the drone components and offboard computational boards: 2.0kg

# Note: Tested on ROS Noetic on Ubuntu 20.04 (Focal Fossa)



## ======================================================================
## Spatial-LM Implementation using realsense d435(i)

- Point Cloud Understanding LLM model, basically an object detection and understanding on processed point cloud data.
- Understand spatial layouts and relationships using language
- more info [here](https://huggingface.co/manycore-research/SpatialLM-Llama-1B)

## ======================================================================

## ====================================================================================================
# PX4-Autpilot and QGC(Drone Mission GUI) setup
## ==========================================================================================
# Drone Navigation Experiments
# Using 3DVFH+* Local Planner for obstacle avoidance and path replanning

- Demo Video (navigation using 3dvfh+* local planner)
 
   https://github.com/user-attachments/assets/145122ba-c00e-4088-ba6d-17a987c4e463

- The 3DVFH+* local planner repo is taken from this repo "https://github.com/PX4/PX4 Avoidance" follow the steps in this repo for installation.
-- >
