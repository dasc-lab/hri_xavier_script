# Human Robot Interaction Outreach Project with Augmented Reality Headset

## System Overview
The camera streams an aerial live video to a hologram on Hololens 2, where the user would have three options to move the ground robot. The first option is to direct the robot to one desired location by air-tapping on the holographic interface. Alternatively, the user would select waypoints in a sequential order in a pre-designed route. Finally, the user could also click buttons that would make the robot perform specified tasks such as ’spin’ or ’advance one meter’. The interface displays the past and predicted route of the robot on the hologram to reduce cognitive load and improve user experience.

The hologram displays the past and predicted optimal trajectories as blue dashed and solid lines respectively. When the user selects a target point by air-tapping on the interface, a red dot will be displayed and when the rover reaches the target coordinate, the red dot would disappear.

## Hardware Components/Prerequisites
The system comprises 
- a Jetson Xavier as the processing unit of the drone
- an Intel RealSense Camera D435  that will be strapped onto the drone. During the testing phase, the camera is strapped onto a beam that faces the ground.
- a Microsoft Hololens that the user will interact with
- a rover that will move according to the user's commands
- [Vicon Coordinate System](https://www.vicon.com/) that provides real time information about the pose of the robot. Note that a minimum of three 'Vicon Pearls would be needed'.


## Software Prerequisites
On Xavier:
- [Robot Operating System 2 (ROS2)](https://docs.ros.org/en/foxy/index.html)
- OpenCV2 and Numpy
- [Realsense libraries](https://github.com/IntelRealSense/librealsense)
  
On rover:
- PX4-Autopilot-Quad

## Socket Communication
