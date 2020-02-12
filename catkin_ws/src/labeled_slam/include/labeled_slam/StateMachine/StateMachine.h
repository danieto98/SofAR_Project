#ifndef STATE_MACHINE_H
#define STATE_MACHINE_H

// ROS includes.
#include "ros/ros.h"
#include "ros/time.h"
#include "labeled_slam/Command.h"
#include "std_srvs/SetBool.h"
#include "std_msgs/Bool.h"

// comment out the following line after rtabmap package has been created!
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


/** @class StateMachine
 *  @brief Implements the interface for the labeled_slam state machine
 *
 *  Uses the c++ 'State'-pattern
 **/
class StateMachine
{
public:
//! Constructor
StateMachine(ros::ServiceClient* client_set_goal,
             ros::ServiceClient* client_set_label,
             ros::ServiceClient* client_activate_path_following,
             ros::ServiceClient* client_activate_driving);

//! Destructor
~StateMachine();

//! Callback function for text_command subscriber
void callback_command(const labeled_slam::Command::ConstPtr& msg);
//! Callback function for goal_reached subscriber
void callback_goal_reached(const std_msgs::Bool::ConstPtr& msg);

private:
ros::ServiceClient* client_set_goal_;               /**< Service client to call service of rtabmap_ros/SetGoal */
ros::ServiceClient* client_set_label_;              /**< Service client to call service of rtabmap_ros/SetLabel */
ros::ServiceClient* client_activate_path_following_;/**< Service client to call service (of Activator1) which activates path following */
ros::ServiceClient* client_activate_driving_;       /**< Service client to call service (of Activator2) which activates driving */


BaseState* state_;                                  /**< Member, which contains the current state object of derived state class (POLYMORPHISM!) */

// Comments on functions in CPP-File!
void change_state (BaseState * state);

// define friend classes in order to acess functions like change_state
friend class State_DRIVING;
friend class State_LISTENING;
friend class State_GO_TO;
};

#endif // STATE_MACHINE_H
