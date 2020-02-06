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



##### SPEECH RECOGNITION REQUIREMENTS #####

For the Speech Recognition Library, you can follow the guidelines at:  https://pypi.org/project/SpeechRecognition/ .
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

