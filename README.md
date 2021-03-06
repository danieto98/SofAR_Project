# Hands on with SLAM

## Objective of the Project

This project aims to enhance the capabilities of a Husqvarna automower by combining Simultaneous Localization and Mapping (SLAM) and voice recognition to provide labeling of points in a generated map of the environment. The map can later be traversed by setting labeled points as desired goals and using a path planning algorithm to reach those points starting from the current estimated position of the robot.

The robot is expected to navigate an area, through smartwatch teleoperation, and acquire labels for objects when a voice command is sent. Then, it is expected to move between labeled objects once voice commands are given through the microphone.

This project has been developed using the [Robot Operating System (ROS)](https://www.ros.org/).

We have used the following hardware components:
* Husqvarna 430x automower with a Raspberry Pi installed as explained in section 7.3 [here](https://github.com/HusqvarnaResearch/hrp/blob/master/Startup%20Guide%20HRP.pdf)
* LG Smart Watch with IMU stream application installed
* Microsoft Kinect for Xbox 360 attached to the top of the automower
* Microphone
* Wi-Fi router
* PC running ROS Kinetic or Melodic on a Linux distribution

## Architecture of the System

The architecture can be summarized using the following UML diagram:
<p align="center">
<img src="https://github.com/danieto98/SofAR_Project/blob/master/UML.png">
</p>

To see the Doxygen documentation, click <a href="https://danieto98.github.io/SofAR_Project/catkin_ws/src/labeled_slam/docs/html/index.html">here</a>.

The Kinect driver (freenect_camera) provides an image, a depth image and the camera information for the device. All of this data is synchronized into a single topic using the rtabmap_ros/rgbd_sync nodelet. This is later fed to both the rtabmap and rgbd_odometry nodes. The latter computes odometry from the current image and point cloud visualized by the Kinect. The results from this node and the synchronized Kinect data are fed into the rtabmap node which generates an estimate of the current map and computes the position of the robot as a tf using an RGBD SLAM approach.

At the same time, the main_speech_controller node listens to microphone and keyboard inputs and outputs valid commands to the logic node, which uses a state machine to change its behavior according to the given command. This node consequently publishes messages to either activator_1 or activator_2, which output the messages they receive as input to the velocity_forwarder node in case the received input from the logic node is true. The velocity_forwarder node lets all input messages through as velocity commands to the Husqvarna's driver (am_driver_legacy).

The logic node also sets labels to the current position upon command by issuing the set_label() service call to the rtabmap node. In case a "go to" command is issued, it uses the set_goal() service call to the rtabmap node instead, which will consequently output a path for the robot to follow from its current position to reach that goal. This path is used by the path_folower, who listens to the tf of the current position and outputs velocity commands (activated by the logic node using activator_1) needed to reach that goal.

When not following a given path, the robot is controlled by using certain gestures captured by the smartwatch. The IMU data from the watch is received by the gb_controller node, which outputs velocity commands to the robot (activated by the logic node using activator_2).

## Description of the System's Architecture

### Activator Modules
The system architecture requires 2 activator modules, that were mainly developed by Filip and Roberto. Their source code was written completely from scratch, because they strongly interact with the main logic, and are very specific for this project. This is why no additional software or libraries need to be installed and no specific hardware is necessary to run the nodes.

These 2 nodes are used to activate the two different types of motion that the Husqvarna lawnmower can have. As previously described, the robot can move when it receives input from the LG Smartwatch, as DRIVING_MODE, or it can follow a path autonomously to reach a pre-defined goal, previously identified as GO_TO_MODE.

Activator1 receives robot velocity inputs and passes it to the velocity forwarder (the node that actually sends velocity to the robot) when the state machine enters the GO_TO_MODE; Activator2 behaves similarly when in DRIVING_MODE. The state machine activates both activators through a Service call. The activators cannot overlap, as they are activated only in these two modes, and the state machine operation prevents activation inaccuracies.


#### Activator 1

Input:

* path/cmd_vel (geometry_msgs/Twist)

Output:

* ac1/cmd_vel (geometry_msgs/Twist)    

Advertised service:

* activate_path_following (std_srvs/SetBool)   


#### Activator 2

Input:

* gbc/cmd_vel (geometry_msgs/Twist)

Output:

* ac2/cmd_vel (geometry_msgs/Twist)

Advertised service:

* activate_driving (std_srvs/SetBool)


### Path Follower Module
The path follower calculates how to follow the path it receives from the rtabmap_ros node. It will receive the entire path, which is an array of poses, but it will only try to go to the next one. First of all, the node checks whether the robot is at the desired pose and distance of the path by using the functions proximity_check() and angle_check(). If the robot is not at the desired pose, it calculate the distance and the angular difference between the two. To calculate the distance, it takes the position of the robot (Point) and the position of the desired pose (Vector3) and subtract them. The path follower publishes velocities as the desired rate, which are regulated through a simple PID controller, that regulates the velocity according to the distance to the target.

Input:

* local_path (nav_msgs/path)   

Output:

* path/cmd_vel (geometry_msgs/Twist)

Required tf:

* /map to /base_link      


### Logic-node
The source code of this module was written completely from scratch because it implements the main logic, which is very specific for this project. This is why no additional software or libraries need to be installed and no specific hardware is necessary to run this node. To run this module, simply type: rosrun labeled_slam logic_node

Input:  


* text_command (labeled_slam/Command)
* goal_reached (std_msgs/Bool)

Output:

* set_goal (rtabmap_ros/SetGoal)
* set_label (rtabmap_ros/SetLabel)
* activate_path_following (std_srvs/SetBool)
* activate_driving (std_srvs/SetBool)

### Speech Recognition Module
The final objective of this node is to properly receive a standard audio input from the user, analyze its content and then generate an output that the state machine can work with to change the state of the system. The Speech recognition module deals with the programmatic design for accomplishing this requirement of the project. After the dependencies have been installed, the module can be run by typing: rosrun labeled_slam command_recognition.py

Input:

* Audio signal from the microphone (hardware)
* Keyboard inputs (hardware)

Output:

* text_command(msg/command)

### Velocity Forwarder Module
This node subscribes to the output topics of "activator1" and "activator2" nodes, and publishes any incoming data to the Husqvarna robot. Having two nodes publishing data on the same topic can be problematic, so the Velocity Forwarder node simply merges all the incoming messages into the same output, avoiding any problems that this procedure could create. The module can be run, simply by typing: rosrun labeled_slam velocity_forwarder

Input:

* ac1/cmd_vel (geometry_msgs/Twist)
* ac2/cmd_vel (geometry_msgs/Twist)

Output:

* cmd_vel (geometry_msgs/Twist)

### Simulator Module
This module is run by the Gazebo Simulator. It simulates our Husqvarna robot with a Kinect device attached to it. Its purpose is to run simulations without being in the laboratory. It can be used to acquire data from a virtual Kinect sensor and/or to represent the theoretical movements that the robot should do. The module can be run, simply by typing: roslaunch am_gazebo am_gazebo_hrp.launch GUI:=true

Input:

* cmd_vel (geometry_msgs/Twist)

Output:

* /rgb/image (sensor_msgs/Image)
* /rgb/camera_info (sensor_msgs/CameraInfo)
* /depth/image (sensor_msgs/Image)
* /camera/depth/camera_info (sensor_msgs/CameraInfo)
* /camera/depth/points (sensor_msgs/PointCloud2)


## Installation and System Testing

### Requirements

#### ROS

You must have a working ROS installation. For this project to work, we recommend using either the ROS Kinetic Kame or Melodic Morenia distributions under Ubuntu 16 or 18 respectively; these are the ones we have used during our development and testing, there is no assurance everything will work under a different distribution. Make sure you install the full desktop version.

* You can find instructions on how to download ROS Kinetic [here](http://wiki.ros.org/kinetic/Installation).
* For downloading ROS Melodic, you can find instructions [here](http://wiki.ros.org/melodic/Installation).

#### rtabmap_ros

This is a ROS wrapper for the rtabmap libary which we will be using for SLAM and path planning.

If you are a ROS Kinetic user, install it with:
```
sudo apt-get install ros-kinetic-rtabmap ros-kinetic-rtabmap-ros
```

If you are a ROS Melodic user, use this one instead:
```
sudo apt-get install ros-melodic-rtabmap ros-melodic-rtabmap-ros
```

#### Libfreenect

This library provides drivers for the Microsoft Kinect.

You will first need to clone the libfreenect library from source. Open up a terminal in your desired directory and run:
```
git clone https://github.com/OpenKinect/libfreenect
cd libfreenect
```

Before installing the source, modify the CMakeLists.txt file in the repository's root directory by adding the following line:
```
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS}  -std=c++11")
```

Now make and install the library:
```
mkdir build
cd build
cmake -L  ..
make
sudo make install
```

#### Speech Recognition Library

While installing the speech recognition library, make sure that you always use Python 2 (and thus pip2) instead of Python 3, as the latter is not supported by ROS Kinetic. As we are using a microphone as input, make sure you install PyAudio as well.

##### ROS Melodic Users
For ROS Melodic users, you can find installation instructions for both of these libraries [here](https://pypi.org/project/SpeechRecognition/).

##### ROS Kinetic Users
For ROS Kinetic users, the suggested installation and version requirements might not work. To avoid this problem, we suggest to:

* Download PyAudio-0.2.11.tar.gz [here](https://pypi.org/project/PyAudio/).
* Move the file to the the Python2.7 package folder (/usr/local/lib/python2.7/dist-packages), you will need sudo privileges.
* In that directory, run:
```
tar xvf PyAudio-0.2.11.tar.gz
cd PyAudio-0.2.11
sudo python setup.py install
```

##### Installation Test

In order to test the installation as a Python package, input the following command:
```
python -m speech_recognition
```

### Installation

Follow the instructions below for both the Raspberry Pi (if it is the first time using it on the Husqvarna) and your PC.
If you have trouble accessing the Raspberry Pi, take a look at the [preliminary steps](#preliminary-steps) section.

#### On the Raspberry Pi

Create a new catkin workspace on the home directory:
```
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/
catkin_make
```

Install Git Lfs to handle large files, if you haven't done so yet:
```
git lfs install
```

Navigate to the src directory and clone the Husqvarna driver repository:
```
cd ~/catkin_ws/src
git clone https://github.com/HusqvarnaResearch/hrp
```

Make the catkin workspace:
```
cd ~/catkin_ws
catkin_make
```

See which serial port is currently connected to the Husqvarna (it should be something similar to /dev/ttyACM0):
```
ls /dev | grep tty
```
If more than one serial port shows up, pick the most similar one to the one suggested above. You will have to test which one of them works later on and return here to perform the following step in case the first one you chose was not the one.

Edit the launchfile and change the serial port to the one in use:
```
nano ~/catkin_ws/src/hrp/am_driver_legacy/launch/automower_legacy.launch
```

#### On your PC

Clone this repository:
```
cd ~
git clone --recurse-submodules -j8 https://github.com/danieto98/SofAR_Project.git
```

Make the catkin workspace:
```
cd ~/SofAR_Project/catkin_ws
catkin_make
```
### Running the Project

#### Preliminary Steps

##### Turning on the Husqvarna

* Flip on the two switches on the back of the automower (left, then center).
* Open the lid on top of the robot by pressing the red STOP button.
* Enter the password on the keypad
* Click on "Menu"
* Press 7 and 9 simultaneously for about two seconds to open up a special menu item (a wrench)
* Open up the special menu and select "Special Settings" at the bottom
* Tick the "Override loop detection" option by pressing "OK"
* Press "OK" once more and navigate back to the main menu
* Close the lid on top of the robot

##### Find Your Computer's IP Address
* Run the following command: `ifconfig`
* Find the interface corresponding to your current connection (Ethernet or Wi-Fi), make sure the address is not localhost(127.0.0.1)
* Under that interface, note down the IP address which follows "inet addr:"
* For every terminal you open up, immediately run the following command `export ROS_IP=<your_ip>` by substituting `<your_ip>` with the IP address obtained above. Note that you will have to use this command in every terminal you open up in your PC. Alternatively, you can add the command to the bottom of your .bashrc file. After doing so, close the terminal and open it up again.

#####  Set the ROS Master on the Raspberry Pi
* SSH into the Pi using its IP address: `ssh <pi_username>@<pi_ip>`
* Enter the password
* Set the ROS master to your machine:
```
export ROS_MASTER_URI=http://<your_ip>:11311
```

#### Running the nodes
* Open up a terminal on your machine and run roscore by using the command `roscore`
* On the terminal connected to the Raspberry Pi via ssh, run the following commands:
```
sudo chmod 666 /dev/ttyACM0
roslaunch am_driver_legacy automower_legacy.launch
```
* In a separate terminal, navigate to the repository's catkin workspace directory `cd ~/SofAR_Project/catkin_ws`
* Make the repository if you haven't done so yet by issuing `catkin_make`
* Source the files using `source devel/setup.bash`
* Open up the IMU app on the LG smartwatch, set the IP address to that of your machine and the port number to "11311".

##### Whole Project

If you intend to run the entire project with SLAM being done online (not recommended at this stage of the project, maybe in the future) and Kinect data being saved for later inspection, run:
```
roslaunch labeled_slam test.launch
```

##### Recording/Running Rosbag of Kinect Data Only

First, record a bag of the Kinect data:
```
roslaunch labeled_slam record_bag.launch
```

After recording the bag, kill the processes and run the following command to create the map offline using the recorded data (edit the launchfile to point to the name of your bagfile):
```
roslaunch labeled_slam test_bag.launch
```

##### Recording/Running Rosbag of Kinect Data and Labeling

First, record a bag of the Kinect data and the labels set using voice recognition:
```
roslaunch labeled_slam labels_bag.launch
```

After recording the bag, kill the processes and run the following command to create the map offline using the recorded data (edit the launchfile to point to the name of your bagfile):
```
roslaunch labeled_slam test_bag_labels.launch
```

##### Running the Project in Localization Mode

After having created a map by running one of the previous bagfiles, kill the process and the map will be saved onto rtabmap's long-term memory.

Run the following command to run the project (restricted to DRIVING and GO_TO modes, the latter only if the second method of bag recording and playing above was used):
```
roslaunch labeled_slam test_localize.launch
```
## Tests and Results

To provide the interested party with a visual guide of the test and experiments concerning this project, we here propose three videos that show the physical tests made in the Lab. These video provide a brief guide to our architecture and the physical implementation of the architecture. 

-Rtabmap GUI: https://youtu.be/6lu6xGg_Bjs
-General Test: https://youtu.be/RyLhx6saB6o
-Robot Stuck on Path Following: https://youtu.be/yplDEHOiaA8

## Report

This is the link to the report: https://drive.google.com/drive/folders/1F-D1oRu5Ioa_JKwIRIPIFAkoQlCVzCDN

## Authors

* Filip Hesse: filip_hesse@yahoo.de
* Justin Lee: leej1996@gmail.com
* Daniel Nieto: danieto98@gmail.com
* Roberto Canale: robyrugby95@gmail.com
* Steven Palma Morera: imstevenpm.study@gmail.com
* Josep Rueda: rueda_999@hotmail.com

## References

[GitHub README template.](https://github.com/EmaroLab/GitHub_Readme_Template)
