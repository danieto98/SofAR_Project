#include "State_LISTENING.h"
#include "State_GO_TO.h"
#include "State_DRIVING.h"
#include "StateMachine.h"


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
