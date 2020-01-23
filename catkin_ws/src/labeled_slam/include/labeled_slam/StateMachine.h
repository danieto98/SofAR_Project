#ifndef STATE_MACHINE_H
#define STATE_MACHINE_H

// ROS includes.
#include "ros/ros.h"
#include "ros/time.h"

// Custom message includes. Auto-generated from msg/ directory.
//#include "node_example/node_example_data.h"

// Dynamic reconfigure includes.
//#include <dynamic_reconfigure/server.h>
// Auto-generated from cfg/ directory.
//#include <node_example/node_example_paramsConfig.h>

using std::string;

class StateMachine
{
public:
//! Constructor.
StateMachine();

//! Destructor.
~StateMachine();


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
