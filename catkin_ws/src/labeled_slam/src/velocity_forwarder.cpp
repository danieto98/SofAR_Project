#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include "std_msgs/String.h"
#include <sstream>

// @brief This node collects messages and sends them to the robot of the labeled_slam
//project. It subscribes to both topics coming from the, "activator_1" and
//"activator_2" nodes and publishes any incoming data to the Husqvarna robot.

geometry_msgs::Twist msg;
ros::Publisher pub;

/**
The Callback functions, mainly publish the received messages on the topic chosen in
the "int main"
The only difference, as said before, is the beginning of the display message:
"Subscriber 1 velocities:" and "Subscriber 2 velocities:"
ROS_INFO_STREAM outputs the received messages through the terminal
*/

void Callback_activator1(const geometry_msgs::Twist &msg)
{

  ROS_INFO_STREAM("Subscriber 1 velocities:" << " linear=" << msg.linear << " angular=" << msg.angular);
  pub.publish(msg);
}

void Callback_activator2(const geometry_msgs::Twist &msg)
{
  ROS_INFO_STREAM("Subscriber 2 velocities:" << " linear=" << msg.linear << " angular=" << msg.angular);
  pub.publish(msg);
}

int main(int argc, char **argv)
{
  //We initialise the node with the name "velocity_forwarder"
  ros::init(argc, argv, "velocity_forwarder");
  
  ros::NodeHandle nh;
/**
We subscribe to the "ac1/cmd_vel" and "ac2/cmd_vel" topics, which are the topics where the nodes "activator_1" and "activator_2" publish messages respectively.
Everytime we receive a message we will execute the callback functions "Callback_activator1" and "Callback_activator2" depending on the topic we receive the incoming message.
They both do mainly the same, so we could use the same callback function for both, but we have split them so we can see which topic are we reading from on the terminal.
*/
  ros::Subscriber sub1 = nh.subscribe("ac1/cmd_vel", 1, &Callback_activator1);
  ros::Subscriber sub2 = nh.subscribe("ac2/cmd_vel", 1, &Callback_activator2);
  
  ros::Rate rate(2);

  while(ros::ok)
  {
    //We set as publishing topic the "cmd_vel", which is the one used by the robot itself
    pub = nh.advertise<geometry_msgs::Twist>("cmd_vel", 1);
    ros::spinOnce();
  }

}

