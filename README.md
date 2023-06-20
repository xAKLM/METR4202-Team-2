# METR4202-Team-Project Team 2
This package has been made specifically for the operation of Team 2's robot arm for the METR4202 team project. The arm design is based off of a SCARA robot, 
allowing for easy calculation of the inverse kinematics. The robot arm works in tandem with a tripod mounted ximea camera which detects blocks by fiducial markings and colour. 
The coordinates of the blocks are then calculated and sent to the arm to move, grab and sort the blocks. The software is run off of a raspberry pi which communicates with both 
the robot arm and the camera and the code structure is implemented through ROS.

The purpose of this relatively small robot is to be a proof of concept for a upscaled version that can be used to sort luggage at an airport.

### Additional Packages
These will need to be installed for the operation of the robot:

* [modern_robotics](https://github.com/NxRLab/ModernRobotics)
* [dynamixel_interface](https://github.com/csiro-robotics/dynamixel_interface#readme)
* [pigpio](https://abyz.me.uk/rpi/pigpio/download.html)
* [ximea](https://github.com/uqmvale6/metr4202_ximea_tutorial)

### Publishers
* desired_joint_states : publishes the desired joint states to the JointState topic

### Subscribers 
* fiducial_vertices : gets the fiducial vertices
* /ximea_cam/image_raw : gets the raw image data from the ximea camera
* /tf & /fiducial_transforms : gets fiducial transform data

### Python Scripts
* TFLISTENER.py : Initialises subscribers for reading and processing image data from the ximea cameras. Some subscribers provide data for colour detection while others provide data for transformation from detected blocks to the base frame of the robot.
* JOINT_STATE_PUBLISHER.py : Initialises publisher to the JointState topic and publishes the desired joint states of the robot arm.
* MAIN.py : Imports/calls the other scripts and handles the main logical sequence of the code. Controls where robot moves end effector based on camera data and controls the actuation of the end effector.

# Usage

### Gazebo Simulation
A gazebo simulation of the robot can be launched using the commands:

```bash
source devel/setup.bash
roslaunch Robot_description gazebo.launch
```
&
```bash
source devel/setup.bash
roslaunch Robot_description controller.launch
```

The gazebo simulation can then be controlled using ```rqt_gui```

### Robot Operation
To start operation of the robot arm & camera, you need to run the following commands on the raspberry pi:
```bash
roslaunch ximea_ros_cam example_cam.launch
roslaunch dynamixel_interface dynamixel_interface_controller.launch
roslaunch aruco_detect aruco_detect.launch
roscd Robot_description/src/
python MAIN.PY
```
