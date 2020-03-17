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
bool angle_check(float goal_angle, tfScalar yaw);
std::vector<geometry_msgs::PoseStamped> current_path;
std::vector<geometry_msgs::PoseStamped>::iterator it;
///Goal position on map
geometry_msgs::Point goal_position;
///Goal orientation on map
geometry_msgs::Quaternion goal_orientation;
///Velocity sent to robot driver
geometry_msgs::Twist velocity_to_publish;

float tolerance_angle, tolerance_dist;

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

        ros::Rate loop_rate(1);

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

        float inc_x, inc_y, direction, angle_to_goal, angle_difference, K_lin, K_ang;
        tf::StampedTransform my_transform;
        tolerance_angle = 0.5;
        tolerance_dist = 0.5;
        K_lin=0.1;
        K_ang=0.2;
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

                if (current_yaw < 0)
                {
                        current_yaw = current_yaw + 2*M_PI;
                }


                inc_x = goal_position.x - current_position.getX();
                inc_y = goal_position.y - current_position.getY();

                angle_to_goal = atan2 (inc_y, inc_x);
                angle_difference = current_yaw - angle_to_goal;

                if(angle_difference < 0) {
                        angle_difference = -angle_difference;
                }

                //determine the rotation direction
                if (angle_to_goal < current_yaw) {
                        if((2*M_PI - current_yaw + angle_to_goal) < (current_yaw - angle_to_goal)) {
                                direction = 1;
                        } else {
                                direction = -1;
                        }
                } else {
                        if( (2*M_PI - angle_to_goal + current_yaw) < (angle_to_goal - current_yaw)) {
                                direction = -1;
                        } else {
                                direction = 1;
                        }
                }


                //implementing a PID controller for speed, speed depends on target distance. In case it doesnt work, just use 0 and 1s

                //IF WE ARE NOT THERE YET
                if(!angle_check(angle_to_goal, current_yaw)) {  //ANGLE IS NOT GOOD ENOUGH
                        //ROTATION & STRAIGHT
                        ROS_INFO("ROTATE");
                        ROS_INFO("Angle_to_goal = %f\n Current_yaw = %f", angle_to_goal, current_yaw);
                        //velocity_to_publish.linear.x = K_lin*sqrt(pow(inc_y, 2) + pow(inc_x, 2));
                        velocity_to_publish.linear.x = 0;
                        velocity_to_publish.angular.z = K_ang * direction * abs(angle_to_goal - current_yaw);

                        if (abs(velocity_to_publish.angular.z) > 0.45)
                        {
                                velocity_to_publish.angular.z = direction * 0.45;
                        }

                } else if (!proximity_check(goal_position, current_position)) { //DISTANCE IS NOT GOOD ENOUGH
                        //GO STRAIGHT
                        ROS_INFO("GO_STRAIGHT");
                        ROS_INFO("Goal_position = %f\n Current_position = %f", goal_position, current_position);
                        velocity_to_publish.linear.x = K_lin * sqrt(pow(inc_y, 2) + pow(inc_x, 2));

                        velocity_to_publish.angular.z = 0.0;
                        if (velocity_to_publish.linear.x >0.9)
                        {
                                velocity_to_publish.linear.x  = 0.9;
                        }
                        /* This was useless I believe! it's an impossible scenario" -> inc_x and inc_y are squared!
                           if (velocity_to_publish.linear.x <-0.9)
                           {
                                velocity_to_publish.linear.x  = -0.9;
                           } */
                }
                else{ //IF WE ARE THERE THEN STOP & ITERATE TO NEXT GOAL POSITION
                        velocity_to_publish.linear.x = 0.0;
                        velocity_to_publish.angular.z = 0.0;
                        if(it != current_path.end()) {
                                ++it;
                                goal_position = it->pose.position;
                                goal_orientation = it->pose.orientation;
                        }
                }

                //publish velocity to robot
                ROS_INFO("Velocity:\n X_lin = %f\n Z_ang = %f\n",velocity_to_publish.linear.x, velocity_to_publish.angular.z);
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


        current_path = received_path->poses;
        it = current_path.begin();
        goal_position = it->pose.position;
        goal_orientation = it->pose.orientation;
        for (auto itr=current_path.begin(); itr!=current_path.end(); itr++) {
                ROS_INFO("Path[%ld] [x;y]: [%f ; %f]\n", itr-current_path.begin(), itr->pose.position.x, itr->pose.position.y );
        }
}
/** Proximity Check
 * The function checks whether the  robot is close enough to the target position and returns a bool.
 * It takes as input and uses the xy coordinates of the goal position and current positions.
 * If the robot is close enough, it returns a true, otherwise returns a false.
 */

bool proximity_check(geometry_msgs::Point goal, tf::Point current){
        //checking if the robot is close/has almost reached to my goal
        if( sqrt( pow((goal.x - current.getX()),2) + pow((goal.y - current.getY()),2)) <tolerance_dist  ) {
                return true;
        } else {
                return false;
        }
}
bool angle_check(float goal_angle, tfScalar yaw){
        //checking if the robot is close/has almost reached to my goal
        if( (abs(goal_angle - yaw) <= tolerance_angle)) {
                return true;
        } else {
                return false;
        }
}
