#include "StateMachine.h"
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "labeled_slam/Command.h"

//#include <sstream>

int main(int argc, char **argv)
{
        ros::init(argc, argv, "logic_node");

        //Create state machine
        StateMachine state_machine;

        ros::NodeHandle n;
        ros::Subscriber sub = n.subscribe("text_command", 1000, &StateMachine::callback, &state_machine);

        ros::Publisher chatter_pub = n.advertise<labeled_slam::Command>("chatter", 1000);

        /*
           std_msgs::String msg;
           std::stringstream ss;
           ss << "hello world " << count;
           msg.data = ss.str();
           ROS_INFO("%s", msg.data.c_str());
           chatter_pub.publish(msg);
         */

        ros::spin();

        return 0;
}
