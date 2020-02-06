#include "StateMachine.h"
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "labeled_slam/Command.h"

//#include <sstream>

int main(int argc, char **argv)
{
        ros::init(argc, argv, "logic_node");

        ros::ServiceClient client_set_goal = n.serviceClient<rtabmap_ros::SetGoal>("set_goal");

        //Create state machine
        StateMachine state_machine(*client_set_goal);

        ros::NodeHandle n;
        ros::Subscriber sub_command = n.subscribe("text_command", 1000, &StateMachine::callback_command, &state_machine);
        ros::Subscriber sub_goal_reached = n.subscribe("goal_reached", 1000, &StateMachine::callback_goal_reached, &state_machine);

        ros::Publisher chatter_pub = n.advertise<labeled_slam::Command>("chatter", 1000);



        ros::spin();

        return 0;
}
