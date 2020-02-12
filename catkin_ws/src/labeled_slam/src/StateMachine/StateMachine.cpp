#include "StateMachine.h"
#include "State_DRIVING.h"
#include "State_GO_TO.h"
#include "State_LISTENING.h"
#include <iostream>


/**
 * @brief Constructor of StateMachine Class
 **/
StateMachine::StateMachine(ros::ServiceClient* client_set_goal,
                           ros::ServiceClient* client_set_label,
                           ros::ServiceClient* client_activate_path_following,
                           ros::ServiceClient* client_activate_driving)
        : state_(new State_DRIVING(this) )
        , client_set_goal_( client_set_goal )
        , client_set_label_( client_set_label )
        , client_activate_path_following_( client_activate_path_following )
        , client_activate_driving_( client_activate_driving )
{
}

/**
 * @brief Destructor of StateMachine Class
 **/
StateMachine::~StateMachine()
{
        delete state_;
}

/**
 *  @brief Callback-function interpreting command from command-recognition node
 *
 *  Allowed commands are "drive", "go to", "label", "listen"
 *  commands go_to and label are using an argument. For the other commands, the
 *  argument is just ignored.
 *  According function of the state_member are called on each command
 *  Depending on the true object-type in the state_-variable, the function of one
 *  specific state is called (POLYMORPHISM!)
 **/
void StateMachine::callback_command(const labeled_slam::Command::ConstPtr& msg)
{
        string command = msg->command;
        string argument = msg->argument;

        if (msg->command.compare("drive") == 0) //strings are equal!
        {
                state_->drive(this);
        }
        else if (msg->command.compare("listen") == 0) //strings are equal!
        {
                state_->listen(this);
        }
        else if (msg->command.compare("go to") == 0) //strings are equal!
        {
                state_->go_to(this, msg->argument);
        }
        else if (msg->command.compare("label") == 0) //strings are equal!
        {
                state_->label(this, msg->argument);
        }
        else
        {
                ROS_INFO("wrong command: %s", command.c_str());
                ROS_INFO("Allowed commands are 'drive', 'go to', 'label', 'listen'");
        }
}

/**
 * @brief Callback-function for the subscribed topic goal_reached
 **/
void StateMachine::callback_goal_reached(const std_msgs::Bool::ConstPtr& msg)
{
        if(msg->data == true) // A boolean with value TRUE must be sent
        {
                state_->goal_reached(this);
        }
}

/**
 * @brief Change the state of state StateMachine
 *
 *   Important: needs to be called with new new_state
 *   To avoid memory leaks, call delete(this) after calling this function
 *   from the old state
 **/
void StateMachine::change_state(BaseState * state)
{
        state_ = state;
}
