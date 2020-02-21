#include "State_DRIVING.h"
#include "State_LISTENING.h"
#include "StateMachine.h"

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
        ROS_INFO("srv.request.data %d.\n",srv.request.data);
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
