# Plug &amp; play program for controlling Tello drone with aruco tags
The goal of this project is to use ROS to controll the DJI Tello drone. By previously determining the ArUco tag's pose through the drone's camera, we will be able to control the movement of our drone.
<p align="center">
 <img align="center" src="assets/ProjectOverview.png" width="350" /> 
</p>

The directions will be provided to the drone once the camera detects the ArUco tag. 

Depending on the ArUco tag's detected location, the list of the possible directions and actions are:
- Landing - *the landing of the drone can be done only by **ID 10** tag*
> <img align="center" src="assets/Landing.png" height="200" />

- Moving forward and backward - *that will cause the drone to hover between the 0.4 and 0.6 m distance*
> <img align="center" src="assets/ForwardBackward.png" height="200" />


- Moving up and down
> <img align="center" src="assets/UpDown.png" height="200" />


- Moving left and right
> <img align="center" src="assets/LeftRight.png" height="200" />

<hr style="border:2px solid gray">

## Project requirements:
- Ubuntu (20.04.3) with ROS1 noetic
- DJI Tello drone
- Printed [Chess Board](https://www.mrpt.org/downloads/camera-calibration-checker-board_9x7.pdf) for camera calibration
- Two [ArUco tags](https://chev.me/arucogen/) from 4x4 dictionary, both size 50mm:
  - One for controlling the drone, which can be any ID of the user's choice
  - One for landing (similar to an emergency stop), which **must be ID 10**, as previously mentioned
<hr style="border:2px solid gray">

## Installing
*Disclaimer:* We have tested on Ubuntu 20.04.3 with ROS Noetic with an GeForce GTX 1050 Ti with Python 3.8, whereas the code has not been tested on other systems.
<hr style="border:2px solid gray">

The following steps describe the installation.

1. **Install ROS**

   By following [these](http://wiki.ros.org/noetic/Installation/Ubuntu) instructions. 
   
2. **Create a catkin workspace** (if the user already has one, this step can be skipped). 
   
   To create a catkin workspace, follow these [instructions](http://wiki.ros.org/catkin/Tutorials/create_a_workspace):
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

   - Turn on your drone and connect to Tello's Wi-Fi.
   - Run the following commands to start camera calibration:
   ```
   $ cd ~/catkin_ws
   $ source devel/setup.bash
   $ rosrun tello_ros TelloCameraCalibration.py
   ```
   - Take 8 images for camera calibration by pressing “S” (as *save*) on the keyboard. As a result, the words “Image taken *X”* (X as the number of your image) will be displayed in the terminal.
   - A few moments after the pictures are taken, the calibration parameters will be written in the terminal, as well as in a file named “CamCalibParam.json” that will be automatically created in the “cameraCalibration” folder. 
     - The images used for calibration will be stored in the folder “cameraCalibration/images”, so that the calibration process can be reruned with the same images. In that case, it will be necessary to add word **“False”** in the if statement in line 25, to turn off Tello activation.
2. **Launching the project** 

    For easily launching multiple ROS nodes, run the launch file named “tello_launch.launch”. The launch file will automatically run the roscore and all of the nodes with Rviz for visualization.
   ```
   $ cd ~/catkin_ws/
   $ source devel/setup.bash
   $ roslaunch tello_ros tello_launch.launch  
   ```
   Once the image appears, the drone can be controlled with ArUco tags.
