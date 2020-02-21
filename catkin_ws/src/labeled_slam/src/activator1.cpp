#include <sstream>
#include <stdlib.h>
#include "ros/ros.h"
#include "std_msgs/Bool.h"
#include "std_srvs/SetBool.h"
#include "geometry_msgs/Twist.h"

using namespace std;

//Constants and general stuff
geometry_msgs::Twist velocity_to_publish;
bool activation = false;
void velocity_callback(const geometry_msgs::Twist::ConstPtr& received_velocity);
bool path_following_activate(std_srvs::SetBool::Request& req, std_srvs::SetBool::Response& response);



int main(int argc, char** argv){

        ros::init(argc, argv, "activator1");

        //initializing my node
        ros::NodeHandle node;

        ros::Rate loop_rate(1000);

        ros::Publisher twist_pub = node.advertise<geometry_msgs::Twist>("ac1/cmd_vel", 1000); //publisher for the veolocity forwarder/the robot
        ros::Subscriber twist_sub = node.subscribe("path/cmd_vel", 1000, &velocity_callback); //subscriber for the velocity from the path planner
        ros::ServiceServer bool_serv = node.advertiseService("activate_path_following", path_following_activate); //boolean check to see wether data needs to be sent or not.


        ROS_INFO("Path Following is not active.\n");

// while here dunno why
        while(ros::ok()) {
                if(activation == true)
                        twist_pub.publish(velocity_to_publish);

                ros::spinOnce();
                loop_rate.sleep();
        }

        return 0;
};


bool path_following_activate(std_srvs::SetBool::Request& req, std_srvs::SetBool::Response& response){

        activation  = req.data;// iff activation->data = true

        if (activation == true)
                ROS_INFO("Path Following is active.\n");
        else
                ROS_INFO("Path Following is not active.\n");
        response.success = true;
        response.message = "";
        return true;

}

void velocity_callback(const geometry_msgs::Twist::ConstPtr& received_velocity){
        velocity_to_publish.linear.x = received_velocity->linear.x;
        velocity_to_publish.linear.y = received_velocity->linear.y;
        velocity_to_publish.linear.z = received_velocity->linear.z;
        velocity_to_publish.angular.x = received_velocity->angular.x;
        velocity_to_publish.angular.y = received_velocity->angular.y;
        velocity_to_publish.angular.z = received_velocity->angular.z;
}
