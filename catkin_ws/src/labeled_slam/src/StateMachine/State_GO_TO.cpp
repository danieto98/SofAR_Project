#include "State_GO_TO.h"
#include "State_LISTENING.h"
#include "StateMachine.h"

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
