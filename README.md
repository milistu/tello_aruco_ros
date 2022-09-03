# Plug &amp; play program for controlling Tello drone with aruco tags
In this project, we are controlling the DJI Tello drone with ROS and using its camera to determine the pose of the ArUco marker which will be used to control the movement of our drone.
<p align="center">
 <img align="center" src="assets/ProjectOverview.png" width="350" /> 
</p>

When the camera feed pick’s up the ArUco tag, depending on the location, the directions are given to the drone. The list of directions and actions are:
- Landing - *only tag with **ID 10** will land the drone*
> <img align="center" src="assets/Landing.png" height="200" />

- Move forward and backward - *the drone will howere between the distance of 0.4 and 0.6 m*
> <img align="center" src="assets/ForwardBackward.png" height="200" />


- Move up and down
> <img align="center" src="assets/UpDown.png" height="200" />


- Move left and right
> <img align="center" src="assets/LeftRight.png" height="200" />

<hr style="border:2px solid gray">

## Project requirements:
- Ubuntu (20.04.3) with ROS1 noetic
- Dji Tello drone
- Printed [Chess Board](https://www.mrpt.org/downloads/camera-calibration-checker-board_9x7.pdf) for camera calibration
- Two [ArUco tags](https://chev.me/arucogen/) from 4x4 dictionary, size 50mm:
  - One for controlling the drone, any ID of your choice
  - The other one is for landing (similar to an emergency stop), **must be ID 10**
<hr style="border:2px solid gray">

## Installing
We have tested on Ubuntu 20.04.3 with ROS Noetic with an GeForce GTX 1050 Ti with Python 3.8. The code may work on other systems.
<hr style="border:2px solid gray">

The following steps describe the installation.

1. **Install ROS**

   Follow [these](http://wiki.ros.org/noetic/Installation/Ubuntu) instructions. 
   
2. **Create a catkin workspace** (if you do not already have one). To create a catkin workspace, follow these [instructions](http://wiki.ros.org/catkin/Tutorials/create_a_workspace):
   ```
   $ mkdir -p ~/catkin_ws/src   # Replace "catkin_ws" with the name of your workspace
   $ cd ~/catkin_ws/
   $ catkin build
   ```
3. **Download the project**

   ```
   $ cd ~/catkin_ws/src
   $ gh repo clone MilStu/tello_aruco_ros
   ```
4. **Install python dependencies**

   ```
   $ cd ~/catkin_ws/src/tello_ros
   $ python3 -m pip install -r requirements.txt
   ```
5. **Build**

   ```
   $ cd ~/catkin_ws
   $ catkin build
   ```
<hr style="border:2px solid gray">

## Running
1. **Camera calibration**

   - Turn on your drone and connect to Tello's wifi.
   - Run these commands to start camera calibration:
   ```
   $ cd ~/catkin_ws
   $ source devel/setup.bash
   $ rosrun tello_ros TelloCameraCalibration.py
   ```
   - Take 8 images for camera calibration by pressing “s” (as save) on the keyboard. In the terminal will be printed “Image taken X” (X as the number of your image).
   - After capturing all of the images required wait for the results, it may take a few moments. 
   - You will get calibration parameters printed out in your terminal and also they will be written in a file called “CamCalibParam.json” that is created in the “cameraCalibration” folder. 
     - The images used for calibration will be stored in the folder “cameraCalibration/images”, so if you want, you can rerun the calibration process with the same pictures but you need to put **“False”** in the if statement in line 25, to turn off Tello activation
2. **Launch the project** 

    For easily launching multiple ROS nodes we created a launch file called “tello_launch.launch” that you can run. The launch file will automatically run the roscore and all of the nodes with Rviz for visualization.
   ```
   $ cd ~/catkin_ws/
   $ source devel/setup.bash
   $ roslaunch tello_ros tello_launch.launch  
   ```
   Wait for image to appear and then you can try conntroling your drone with ArUco tags.
