# SofAR_Project

This is a repository developed as a group proect for the course of Software Architecture for Robotics-86805 at the University of Genoa. 

The students involved are:
-_Filip Hesse_
-_Justin Lee_
-_Daniel Nieto_
-_Roberto Canale_
-_Steven Palma Morera_
-_Josep Rueda_

The project supervisors are: 
-_Syed Yusha Kareem_
-_Alessandro Carfì_
-_Antony Thomas_

The goal of the project is to combine SLAM with voice recognition and semantic labeling within a closed environment. 
The mobile robot used is an Husqvarna, connected to a RaspberryPi, Kinect for RGBD data, a microphone, and a Smartwatch to teleoperate it. 

The robot is expected to navigate an area, through smartwatch teleoperation, and acquire labels for objects when a voice command is sent. Then, is expected to move between labelled objects once voice commands are given through the microphone. 

##### Turning on Husqvarna #####
-Flip on the two switches on the back of the automower (left, then center).
-Open the lid on top of the robot by pressing the red STOP button.
-Enter the password: ######
-Click on Menu
-Press 7 and 9 simultaneously for about two seconds to open up a special menu item (a wrench)
-Open up the special menu and select "Special Settings" at the bottom
-Tick the "Override loop detection" option by pressing OK
-Press OK once more and navigate back to the main menu
-Close the lid on top of the robot

##### Find your computer's IP address #####
-Run the following command: "ifconfig"
-Find the interface corresponding to your current connection (Ethernet or Wi-Fi), make sure the address is not localhost(127.0.0.1)
-Under that interface, note down the IP address which follows "inet addr:"
-For every terminal you open up, immediately run the following command "export ROS_IP=<your_ip>" by substituting "<your_ip>" with the IP address obtained above

#####  Set the ROS master on the Raspberry Pi ##### 
-SSH into the Pi using its IP address (currently ############ ): "ssh emarolab@<pi_ip>".
-Enter the password: ###########
-Set the ROS master to your machine: ```export ROS_MASTER_URI=<your_ip>:11311 ```

Start up the required nodes:
-Open up a terminal on your machine and be sure to export your IP as explained above
-Run roscore by using the command "roscore"
-In a separate terminal, navigate to the repository's catkin workspace directory
-Make the repository if you haven't done so yet by issuing "catkin_make"
-Source the files using "source devel/setup.bash"
-Be sure to export your IP as explained above
-Run the gesture based controller: "roslaunch gesture_based_controller hrp_gb_controller.launch"
-On the terminal connected to the Raspberry Pi via ssh, run the following commands:
   ```sudo chmod 666 /dev/ttyACM0```
   ```roslaunch am_driver_legacy automower_legacy.launch```
    
#### LG Smartwatch: Start controlling the robot #####
-Open up the IMU app on the LG SmartWatch
-Set the IP address to that of your machine and the port number to "11311"
-Move your hand to drive the robot


##### Speech Recognition, Installation Testing and Requirements #####

For the Speech Recognition Library, you can follow the installations guidelines at:  https://pypi.org/project/SpeechRecognition/ .
In this project, we use a microphone as input: it therefore necessary to install the PyAudio repository (https://pypi.org/project/PyAudio/)
For ROS Melodic users, the installation should not greate any issue. 
For ROS Kinetic users, the suggested installation and version requirements might not work. To avoid this problem, we suggest to:

-Download PyAudio-0.2.11.tar.gz from https: //pypi.org/project/PyAudio/ .
-Move it to the the Python2.7 package folder (/usr/local/lib/python2.7/dist-packages, you will need sudo privileges). 
-In that directory, run:

```
tar xvf PyAudio-0.2.11.tar.gz
cd PyAudio-0.2.11
sudo python setup.py install
```
For a quick package test, it is possible to test it as a Python package (1) or a ROS package (2):

(1)
```
python -m speech_recognition
```
(2) Open three terminals, RUN "source ./devel/setup.bash" in each of them and then run each of the following in a terminal:
```
roscore
rosrun myspeechrecognition myspeechcommand.py
rostopic echo /speechcommandtopic
```

 ##### Installation of freenect_stack package #####
Use these instructions to install libfreenect
```
cd  ~    
git clone https://github.com/OpenKinect/libfreenect.git    
cd libfreenect    
mkdir build   
cd build
cmake -L ..    
make
sudo make install
```
NOTE: you must edit CmakeLists.txt before running cmake and add the following line:
```
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS}  -std=c++11")
```
Use these instructions to install freenect_stack
```
cd ~/catkin_workspace/src
git clone https://github.com/ros-drivers/freenect_stack.git
cd ..
catkin_make
```
##### Installation of rtabmap_ros package #####
Look at this webpage: https://github.com/introlab/rtabmap_ros#build-from-source
NOTE: We do not need any optional dependencies but steps 0, 2, & 3 are required.

