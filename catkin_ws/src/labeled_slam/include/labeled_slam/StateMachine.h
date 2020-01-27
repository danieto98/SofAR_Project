#ifndef STATE_MACHINE_H
#define STATE_MACHINE_H

// ROS includes.
#include "ros/ros.h"
#include "ros/time.h"
#include "labeled_slam/Command.h"

using std::string;

//forward declare classes
class BaseState;
class State_DRIVING;
class State_LISTENING;
class State_GO_TO;

class StateMachine
{
public:
//! Constructor.
StateMachine();

//! Destructor.
~StateMachine();

void callback(const labeled_slam::Command::ConstPtr& msg);

void change_state (BaseState * state);

void drive();
void listen();
void go_to(string target);
void label(string label);

private:
BaseState* state_;

//! Publish the message.
//void publishMessage(ros::Publisher *pub_message);

};

class BaseState
{
public:
virtual void drive(StateMachine* m) = 0;
virtual void listen(StateMachine* m) = 0;
virtual void go_to(StateMachine* m, string target) = 0;
virtual void label(StateMachine* m, string label) = 0;
};


class State_DRIVING : public BaseState
{
virtual void drive(StateMachine* m);
virtual void listen(StateMachine* m);
virtual void go_to(StateMachine* m, string target);
virtual void label(StateMachine* m, string label);
};

class State_LISTENING : public BaseState
{
virtual void drive(StateMachine* m);
virtual void listen(StateMachine* m);
virtual void go_to(StateMachine* m, string target);
virtual void label(StateMachine* m, string label);
};

class State_GO_TO : public BaseState
{
public:
State_GO_TO(string target);
virtual void drive(StateMachine* m);
virtual void listen(StateMachine* m);
virtual void go_to(StateMachine* m, string target);
virtual void label(StateMachine* m, string label);
private:
string target_;


};

#endif // STATE_MACHINE_H
