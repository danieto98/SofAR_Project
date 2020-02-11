#ifndef STATE_MACHINE_H
#define STATE_MACHINE_H

// ROS includes.
#include "ros/ros.h"
#include "ros/time.h"
#include "labeled_slam/Command.h"
#include "std_srvs/SetBool.h"
#include "std_msgs/Bool.h"

#define STATE_MACHINE_STANDALONE

#ifdef STATE_MACHINE_STANDALONE
  #include "labeled_slam/SetGoalDummy.h"
  #include "labeled_slam/SetLabelDummy.h"
  #define SRV_TYPE_SET_GOAL labeled_slam::SetGoalDummy
  #define SRV_TYPE_SET_LABEL labeled_slam::SetLabelDummy
#else
  #include "rtabmap_ros/SetGoal.h"
  #include "rtabmap_ros/SetLabel.h"
  #define SRV_TYPE_SET_GOAL rtabmap_ros::SetGoal
  #define SRV_TYPE_SET_LABEL rtabmap_ros::SetLabel
#endif


using std::string;

//forward declare classess
class BaseState;
class State_DRIVING;
class State_LISTENING;
class State_GO_TO;

class StateMachine
{
public:
//! Constructor.
StateMachine(ros::ServiceClient* client_set_goal,
             ros::ServiceClient* client_set_label,
             ros::ServiceClient* client_activate_path_following,
             ros::ServiceClient* client_activate_driving);

//! Destructor.
~StateMachine();

void callback_command(const labeled_slam::Command::ConstPtr& msg);
void callback_goal_reached(const std_msgs::Bool::ConstPtr& msg);

void change_state (BaseState * state);

ros::ServiceClient* client_set_goal_;
ros::ServiceClient* client_set_label_;
ros::ServiceClient* client_activate_path_following_;
ros::ServiceClient* client_activate_driving_;

private:
BaseState* state_;

void drive();
void listen();
void go_to(string target);
void label(string label);
void goal_reached();

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
virtual void goal_reached(StateMachine* m) = 0;
};


class State_DRIVING : public BaseState
{
public:
State_DRIVING(StateMachine* m);
~State_DRIVING();
virtual void drive(StateMachine* m);
virtual void listen(StateMachine* m);
virtual void go_to(StateMachine* m, string target);
virtual void label(StateMachine* m, string label);
virtual void goal_reached(StateMachine* m);
private:
ros::ServiceClient* client_activate_driving_;
};

class State_LISTENING : public BaseState
{
public:
virtual void drive(StateMachine* m);
virtual void listen(StateMachine* m);
virtual void go_to(StateMachine* m, string target);
virtual void label(StateMachine* m, string label);
virtual void goal_reached(StateMachine* m);
};

class State_GO_TO : public BaseState
{
public:
State_GO_TO(StateMachine* m, string target);
~State_GO_TO();
virtual void drive(StateMachine* m);
virtual void listen(StateMachine* m);
virtual void go_to(StateMachine* m, string target);
virtual void label(StateMachine* m, string label);
virtual void goal_reached(StateMachine* m);
private:
string target_;
ros::ServiceClient* client_activate_path_following_;


};

#endif // STATE_MACHINE_H
