# Plug &amp; play program for controlling Tello drone with aruco tags
In this project, we are controlling the DJI Tello drone with ROS and using its camera to determine the pose of the ArUco marker which will be used to control the movement of our drone.
<p align="center">
 <img align="center" src="assets/ProjectOverview.png" width="350" /> 
</p>
### Project requirements:
- Ubuntu (20.04.3) with ROS1 noetic
- Dji Tello drone
- Printed [Chess Board](https://www.mrpt.org/downloads/camera-calibration-checker-board_9x7.pdf) for camera calibration
- Two [ArUco tags](https://chev.me/arucogen/) from 4x4 dictionary, size 50mm:
  - One for controlling the drone, any ID you choose
  - The other one is for landing (similar to an emergency stop), **must be ID 10**
