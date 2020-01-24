#ifndef STATE_MACHINE_H
#define STATE_MACHINE_H

// ROS includes.
#include "ros/ros.h"
#include "ros/time.h"
#include "labeled_slam/Command.h"

using std::string;

class StateMachine
{
public:
//! Constructor.
StateMachine();

//! Destructor.
~StateMachine();

void callback(const labeled_slam::Command::ConstPtr& msg);
//! Publish the message.
//void publishMessage(ros::Publisher *pub_message);

//! The actual message.
string message;

//! The first integer to use in addition.
int a;

//! The second integer to use in addition.
int b;
};

#endif // STATE_MACHINE_H
