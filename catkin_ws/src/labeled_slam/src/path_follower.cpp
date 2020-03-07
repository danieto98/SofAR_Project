/** Path Follower Node: Receives the goal position and the current position of the robot, and publishes the Twist velocity to reach that goal.
 */
#include <sstream>
#include <stdlib.h>
#include "ros/ros.h"
#include "std_msgs/Header.h"
#include "geometry_msgs/Point.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Vector3.h"
#include "geometry_msgs/TransformStamped.h"
#include "geometry_msgs/Transform.h"
#include <tf/transform_listener.h>
#include "nav_msgs/Path.h"
#include "tf/LinearMath/Transform.h"
#include "tf/LinearMath/Matrix3x3.h"
#include <string>
#include <array>
#include <cmath>

using namespace std;

//Constants and general stuff
//These current positions and orientations could be vectors!!!!!!
//I don't really know the contents, or what rtabmap pushes out, but we'll have to discuss about it
//still need to fix my current position, i just gotta find it!

void path_callback(const nav_msgs::Path::ConstPtr& received_path);
bool proximity_check(geometry_msgs::Point goal, tf::Point current);

std::vector<geometry_msgs::PoseStamped> current_state;
std::vector<geometry_msgs::PoseStamped>::iterator it;
///Goal position on map
geometry_msgs::Point goal_position;
///Goal orientation on map
geometry_msgs::Quaternion goal_orientation;
///Velocity sent to robot driver
geometry_msgs::Twist velocity_to_publish;



/** The node subscribes to a TF topic, and gets the current position of the robot as a base_link.
 * It receives from the callback the goal position, and publishes the required position as a geometry_msgs::Twist to reach that position.
 * The path follower computes the angle difference between the goal position and the robot orientation.
 * The robot then rotates left or right accordingly, until it is on the same line as the goal orientation.
 * At that moment, the robot moves forward. All "robot movements" are considered as published veolcities as geometry_msgs::Twist messages.
 */


int main(int argc, char** argv){

        ros::init(argc, argv, "path_follower");

        //initializing my node
        ros::NodeHandle node;

        ros::Rate loop_rate(1000);

        ros::Publisher twist_pub = node.advertise<geometry_msgs::Twist>("path/cmd_vel", 1000); //publisher for the veolocity forwarder/the robot
        ros::Subscriber nav_sub = node.subscribe("local_path", 1000, &path_callback); //subscriber for the velocity from the path planner
        tf::TransformListener robot_listener;

        ///Current position of robot
        tf::Point current_position;
        ///Current orientation of robot in quaternion
        tf::Quaternion current_orientation;

        goal_position.x = 0.0;
        goal_position.y = 0.0;
        goal_position.z = 0.0;
        goal_orientation.x = 0.0;
        goal_orientation.y = 0.0;
        goal_orientation.z = 0.0;
        goal_orientation.w = 1.0;

        //initializing speeds at 0
        velocity_to_publish.linear.x = 0.0;
        velocity_to_publish.linear.y = 0.0;
        velocity_to_publish.linear.z = 0.0;
        velocity_to_publish.angular.x = 0.0;
        velocity_to_publish.angular.y = 0.0;
        velocity_to_publish.angular.z = 0.0;

        float inc_x, inc_y, angle_to_goal, K_vel;
        tf::StampedTransform my_transform;

        K_vel=1.25;
        //Need a function to obtain my yaw from my current orientation! Maybe dont need current Quaternion! Depends on how I can compute the orientation of my robot.

        while(ros::ok()) {

                //this is how they do it in the tutorial
                try {
                        robot_listener.lookupTransform ("/base_link", "/map", ros::Time(0), my_transform);
                } catch(tf::TransformException ex) {
                        ROS_ERROR("%s", ex.what());
                        ros::Duration(1.0).sleep();
                        continue;
                }

                //initializng robot current position and orientation
                current_position = my_transform.getOrigin();
                current_orientation = my_transform.getRotation();

                tfScalar current_yaw, current_pitch, current_roll;
                tf::Matrix3x3 mat(current_orientation);
                mat.getEulerYPR(current_yaw, current_pitch, current_roll);
                current_pitch=0;
                current_roll=0;


                inc_x = goal_position.x - current_position.getX();
                inc_y = goal_position.y - current_position.getY();
                angle_to_goal = atan2 (inc_y, inc_x);

                //implementing a PID controller for speed, speed depends on target distance. In case it doesnt work, just use 0 and 1s


                if(!proximity_check(goal_position, current_position)) {
                        if((angle_to_goal - current_yaw) > 0.1 ) {
                                //LEFT ROTATION
                                velocity_to_publish.linear.x = K_vel*sqrt(pow(inc_y, 2) + pow(inc_x, 2));
                                velocity_to_publish.angular.z = K_vel*(angle_to_goal - current_yaw);
                        } else if ((angle_to_goal - current_yaw) < -0.1 ) {
                                //RIGHT ROTATION
                                velocity_to_publish.linear.x = K_vel*sqrt(pow(inc_y, 2) + pow(inc_x, 2));
                                velocity_to_publish.angular.z = K_vel*(angle_to_goal - current_yaw);
                        } else if (proximity_check(goal_position, current_position) && abs(angle_to_goal - current_yaw) <= 0.1) {
                                //STOP
                                velocity_to_publish.linear.x = 0.0;
                                velocity_to_publish.angular.z = 0.0;
                        } else { //GO STRAIGHT
                                velocity_to_publish.linear.x = K_vel*sqrt(pow(inc_y, 2) + pow(inc_x, 2));
                                velocity_to_publish.angular.z = 0.0;
                        }
                } else if(it != current_state.end()) {
                        ++it;
                        goal_position = it->pose.position;
                        goal_orientation = it->pose.orientation;
                }


                //publish velocity to robot
                twist_pub.publish(velocity_to_publish);
                ros::spinOnce();
                loop_rate.sleep();
        }

        return 0;
};

/** Path Callback
 * Callback function for the path follower. The node subscribes to the nav_msgs::Path published by rtabmap.
 * These messages provide the node with the goal position and orientation to be achived when in GO_TO_GOAL MODE.
 * The callback is used to initialize the two gobal variables goal_position and goal_orientation used in the
 */

void path_callback(const nav_msgs::Path::ConstPtr& received_path){
        current_state = received_path->poses;
        it = current_state.begin();
        goal_position = it->pose.position;
        goal_orientation = it->pose.orientation;
}
/** Proximity Check
 * The function checks whether the  robot is close enough to the target position and returns a bool.
 * It takes as input and uses the xy coordinates of the goal position and current positions.
 * If the robot is close enough, it returns a true, otherwise returns a false.
 */

bool proximity_check(geometry_msgs::Point goal, tf::Point current){
        //checking if the robot is close/has almost reached to my goal
        if( ( (goal.x - current.getX()) <0.1 ) &&  ( (goal.y - current.getY()) <0.1 ) ) {
                return true;
        } else {
                return false;
        }
}
