Environment:
Ubuntu 16
ROS Kinetic
python3

Camera:
Orbbec Astra Pro 3D Camera

Work table:
To collect a dataset of hand trajectories reaching for one of the targets, make the grid of the 176 possible targets presented by the 16 × 11 grid fixed on the table, with each target being represented by a 5 × 5 cm2. 
In addition, put the four AR markers placed near the grid edges so that the image and the point cloud allow the detection of the grid location inferred by them.

To run this package:
Download HumanMotionCapture and its dependency packages and put them in ~/catkin_ws/src.
The project dependency packages include 
OpenNI2
ros_astra_camera
ros_astra_launch

Build Packages in a catkin Workspace
build all packages located in ~/catkin_ws/src.
$ cd ~/catkin_ws
$ catkin_make

Run the package
$ source ~/catkin_ws/devel/setup.bash
$ roslaunch src/Human*/launch/controller.launch~ (different from roslaunch "controller.launch", for some reason)
$ roslaunch src/Human*/launch/orbbec*

Note: if the camera is not detected at first, leave it plugged in for a while and restart the launch file.
To test and troubleshooting the camera, use $rosrun rviz rviz

Raspberry PI (not copletely programmed yet):
The Raspberry PI is to send the switch signals with the button. It will be running a small ros node that needs to be initialized via SSH. The Raspberry PI is connected to power source, Ethernet, and optionally a screen, and mouse/keyboard for troubleshooting with a usb cable and bluetooth. 
The led light is always on and should be taped to the tip of a finger. To start recording, press the button to send "on" signal and move hand to target position on the grid. When reaching motion is finished, press the button again to send "off" signal to stop the recording.
