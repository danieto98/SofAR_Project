#include "StateMachine.h"
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

/**
 * @brief Constructor of State_DRIVING
 *
 * Calls service activate_driving (always, when this state is entered)
 * with a TRUE flag
 **/
State_DRIVING::State_DRIVING(StateMachine* m)
        : client_activate_driving_(m->client_activate_driving_)
{
        std_srvs::SetBool srv;
        srv.request.data = true;
        if (!ros::service::exists("activate_driving", true  ))   //Info and return, if service does not exist
        {
                ROS_INFO("activate_driving service does not exist! Driving will not be activated.");
                return;
        }
        client_activate_driving_->call(srv);
}

/**
 * @brief Destructor of State_DRIVING
 *
 * Calls service activate_driving (always, when this state is left)
 * with a FALSE flag
 **/
State_DRIVING::~State_DRIVING()
{
        std_srvs::SetBool srv;
        srv.request.data = false;
        if (!ros::service::exists("activate_driving", true  ))   //Info and return, if service does not exist
        {
                ROS_INFO("activate_driving service does not exist! Driving can not be deactivated.");
                return;
        }
        client_activate_driving_->call(srv);
}

/**
 * @brief Don't do anythin on command drive
 **/
void State_DRIVING::drive(StateMachine* m)
{
        ROS_INFO("Already in driving mode!");
}

/**
 * @brief Switch to listening mode
 **/
void State_DRIVING::listen(StateMachine* m)
{
        ROS_INFO("Switching to listening mode");
        m->change_state( new State_LISTENING() );
        delete this;
}

/**
 * @brief Don't do anythin on command go_to
 **/
void State_DRIVING::go_to(StateMachine* m, string target)
{
        ROS_INFO("Invalid Command 'go to' for driving mode. Expected to receive a listen command first (typing '1')");
}

/**
 * @brief Don't do anything on command label
 **/
void State_DRIVING::label(StateMachine* m, string label)
{
        ROS_INFO("Invalid Command 'label' for driving mode. Expected to receive a listen command first (typing '1')");
}

/**
 * @brief Don't do anything when goal reached is received
 **/
void State_DRIVING::goal_reached(StateMachine* m)
{
        ROS_INFO("Invalid message: Goal should not be reached, when in driving mode!");
}

/**
 * @brief Switch to driving mode
 **/
void State_LISTENING::drive(StateMachine* m)
{
        ROS_INFO("Switching to driving mode");
        m->change_state( new State_DRIVING(m) );
        delete this;
}

/**
 * @brief Don't do anything on command listen
 **/
void State_LISTENING::listen(StateMachine* m)
{
        ROS_INFO("Already in listening mode!");
}

/**
 * @brief Switch to go_to mode
 **/
void State_LISTENING::go_to(StateMachine* m, string target)
{
        ROS_INFO("Switching to go_to mode");
        ROS_INFO("Target: %s", target.c_str() );
        m->change_state( new State_GO_TO(m, target) );
        delete this;
}

/**
 * @brief Call label service
 **/
void State_LISTENING::label(StateMachine* m, string label)
{
        SRV_TYPE_SET_LABEL srv;
        srv.request.node_id = 0; //Means, that label will b set to last node_id
        srv.request.node_label = label;
        if (!ros::service::exists("set_label", true  ))//Info and return, if service does not exist
        {
                ROS_INFO("set_label service does not exist! Labeling unsuccessfull.");
                return;
        }
        m->client_set_label_->call(srv);
}

/**
 * @brief Don't do anything when goal reached is received
 **/
void State_LISTENING::goal_reached(StateMachine* m)
{
        ROS_INFO("Invalid message: Goal should not be reached, when in labeling mode!");
}

/**
 * @brief Constructor of State_GO_TO
 *
 * Always, when this state is entered:
 * (1) Calls service activate_path_following
 *     with a TRUE flag
 * (2) Calls service which sets the goal to the rtabmap
 **/
State_GO_TO::State_GO_TO(StateMachine* m, string target)
        : target_(target)
        , client_activate_path_following_(m->client_activate_path_following_)
{
        // activate path following
        std_srvs::SetBool srv_act;
        srv_act.request.data = true;
        if (!ros::service::exists("activate_path_following", true  ))//Info and return, if service does not exist
        {
                ROS_INFO("activate_path_following service does not exist! Path following will not be activated.");
                return;
        }
        client_activate_path_following_->call(srv_act);

        // set new goal
        SRV_TYPE_SET_GOAL srv_goal;
        //srv.request.node_id = 0; //Not sure about that
        srv_goal.request.node_label = target_;
        if (!ros::service::exists("set_goal", true  ))//Info and return, if service does not exist
        {
                ROS_INFO("set_goal service does not exist! Going to specified location unsuccessfull.");
                return;
        }
        m->client_set_goal_->call(srv_goal);
}

/**
 * @brief Destructor of State_GO_TO
 *
 * Calls service activate_path_following (always, when this state is left)
 * with a FALSE flag
 **/
State_GO_TO::~State_GO_TO()
{
        // deactivate path following
        std_srvs::SetBool srv;
        srv.request.data = false;
        if (!ros::service::exists("activate_path_following", true  ))//Info and return, if service does not exist
        {
                ROS_INFO("activate_path_following service does not exist! Path following can not be deactivated.");
                return;
        }
        client_activate_path_following_->call(srv);
}

/**
 * @brief Don't do anything on command drive
 **/
void State_GO_TO::drive(StateMachine* m)
{
        ROS_INFO("Invalid Command 'drive' for go_to mode. Expected to receive a listen command first (typing '1')");
}

/**
 * @brief Switch to listening mode
 **/
void State_GO_TO::listen(StateMachine* m)
{
        ROS_INFO("Switching to listening mode");
        m->change_state( new State_LISTENING() );
        delete this;
}

/**
 * @brief Don't do anything on command go to
 **/
void State_GO_TO::go_to(StateMachine* m, string target)
{
        ROS_INFO("Invalid Command 'go to' for go_to mode. Expected to receive a listen command first (typing '1')");
}

/**
 * @brief Don't do anything on command label
 **/
void State_GO_TO::label(StateMachine* m, string label)
{
        ROS_INFO("Invalid Command 'label' for go_to mode. Expected to receive a listen command first (typing '1')");
}

/**
 * @brief When goal reached is received, switch back to listening mode (leave this state)
 **/
void State_GO_TO::goal_reached(StateMachine* m)
{
        ROS_INFO("GOAL REACHED! Switching to listening mode");
        m->change_state( new State_LISTENING() );
        delete this;
}
