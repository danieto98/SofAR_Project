m>
#include <stdlib.h>
#include "ros/ros.h"
#include "std_msgs/Bool.h"
#include "geometry_msgs/Twist.h"

using namespace std;

//Constants and general stuff
geometry_msgs::Twist velocity_to_publish;
bool activation = false;
void velocity_callback(const geometry_msgs::Twist& received_velocity);
void driving_callback(const std_msgs::Bool::ConstPtr& activation_msg);

//initializing my node

ros::NodeHandle node;
ros::Publisher  twist_pub = node.advertise<geometry_msgs::Twist>("ac1/cmd_vel", 1000); //publisher for the veolocity forwarder/the robot
ros::Subscriber twist_sub = node.subscribe("gbc/cmd_vel", 1000, &velocity_callback); //subscriber for the velocity from the path planner
ros::Subscriber bool_sub = node.subscribe("activate_driving", 1000, &driving_callback);  //boolean check to see wether data needs to be sent or not.


int main(int argc, char** argv){

  ros::init(argc, argv, "activator1");
  ros::Rate loop_rate(1000);

// while here dunno why
  while(ros::ok()){
    if(activation == true){
      ROS_INFO("Driving Mode is active.\n");
    } else{
      ROS_INFO("Driving Mode  is not active.\n");
    }

  ros::spin();
  }

  return 0;
};





void driving_callback(const std_msgs::Bool::ConstPtr& activation_msg){
    if(activation_msg->data == true){
      twist_pub.publish(velocity_to_publish);
      activation  == activation_msg->data ; // iff activation->data = true
       ;
      } else {
        activation  == activation_msg->data; //activation->data = false

        //break; //or smt like  ros::spin();
      }
}

void velocity_calback(const geometry_msgs::Twist& received_velocity){

    velocity_to_publish.linear.x = received_velocity.linear.x;
    velocity_to_publish.linear.y = received_velocity.linear.y;
    velocity_to_publish.linear.z = received_velocity.linear.z;
    velocity_to_publish.angular.x = received_velocity.angular.x;
    velocity_to_publish.angular.y = received_velocity.angular.y;
    velocity_to_publish.angular.z = received_velocity.angular.z;
}
