#include "StateMachine.h"

StateMachine::StateMachine()
{
}


StateMachine::~StateMachine()
{
} // end ~NodeExample()



void StateMachine::callback(const labeled_slam::Command::ConstPtr& msg)
{
        string command = msg->command;
        string argument = msg->argument;


        // Note that these are only set to INFO so they will print to a terminal for example purposes.
        // Typically, they should be DEBUG.
        ROS_INFO("command is %s", command.c_str());
}    // end publishCallback()
