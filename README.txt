Download packages and put them in ~/catkin_ws/src.
The project package and packages include 
OpenNI2
ros_astra_camera
ros_astra_launch

Build Packages in a catkin Workspace
build all packages located in ~/catkin_ws/src.
$ cd ~/catkin_ws
$ catkin_make

Run the package
$ source ~/catkin_ws/devel/setup.bash
$ roslaunch "controller.launch~" (different from roslaunch "controller.launch", for some reason)
$ roslaunch src/Human*/launch/orbbec*
