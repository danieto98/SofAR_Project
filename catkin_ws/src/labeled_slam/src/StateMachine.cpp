#include "StateMachine.h"
#include <iostream>

StateMachine::StateMachine(ros::ServiceClient* client_set_goal)
        : state_(new State_LISTENING() )
        , client_set_goal_( client_set_goal )
{
}


StateMachine::~StateMachine()
{
        delete state_;
} // end ~NodeExample()



void StateMachine::callback_command(const labeled_slam::Command::ConstPtr& msg)
{
        string command = msg->command;
        string argument = msg->argument;

        if (msg->command.compare("drive") == 0) //strings are equal!
        {
                drive();
        }
        else if (msg->command.compare("listen") == 0) //strings are equal!
        {
                listen();
        }
        else if (msg->command.compare("go to") == 0) //strings are equal!
        {
                go_to(msg->argument);
        }
        else if (msg->command.compare("label") == 0) //strings are equal!
        {
                label(msg->argument);
        }
        else
        {
                ROS_INFO("wrong command: %s", command.c_str());
        }
}

void StateMachine::callback_goal_reached(const std_msgs::Bool::ConstPtr& msg)
{
        if(msg->data == true)
        {
                goal_reached();
        }
}

void StateMachine::change_state(BaseState * state)
{
        state_ = state;
}

void StateMachine::drive()
{
        state_->drive(this);
}

void StateMachine::listen()
{
        state_->listen(this);
}

void StateMachine::go_to(string target)
{
        state_->go_to(this, target);
}

void StateMachine::label(string label)
{
        state_->label(this, label);
}

void StateMachine::goal_reached()
{
        state_->goal_reached(this);
}


State_DRIVING::State_DRIVING()
{
        //

}

void State_DRIVING::drive(StateMachine* m)
{
        ROS_INFO("Already in driving mode!");
}


void State_DRIVING::listen(StateMachine* m)
{
        ROS_INFO("Switching to listening mode");
        m->change_state( new State_LISTENING() );
        delete this;
}

void State_DRIVING::go_to(StateMachine* m, string target)
{
        ROS_INFO("Invalid Command 'go to' for driving mode. Expected to receive a listen command first (typing '1')");
}

void State_DRIVING::label(StateMachine* m, string label)
{
        ROS_INFO("Invalid Command 'label' for driving mode. Expected to receive a listen command first (typing '1')");
}

void State_DRIVING::goal_reached(StateMachine* m)
{
        ROS_INFO("Invalid message: Goal should not be reached, when in driving mode!");
}

void State_LISTENING::drive(StateMachine* m)
{
        ROS_INFO("Switching to driving mode");
        m->change_state( new State_DRIVING() );
        delete this;
}

void State_LISTENING::listen(StateMachine* m)
{
        ROS_INFO("Already in listening mode!");
}

void State_LISTENING::go_to(StateMachine* m, string target)
{
        ROS_INFO("Switching to go_to mode");
        ROS_INFO("Target: %s", target.c_str() );
        m->change_state( new State_GO_TO(target, m->client_set_goal_) );
        delete this;
}

void State_LISTENING::label(StateMachine* m, string label)
{
        ROS_INFO("TODO!! Labeling");
}

void State_LISTENING::goal_reached(StateMachine* m)
{
        ROS_INFO("Invalid message: Goal should not be reached, when in labeling mode!");
}

State_GO_TO::State_GO_TO(string target,ros::ServiceClient* client_set_goal)
        : target_(target)
{
        SRV_TYPE_SET_GOAL srv;
        srv.request.node_id = 0; //Not sure about that
        srv.request.node_label = target_;
        if (client_set_goal->call(srv));
}

void State_GO_TO::drive(StateMachine* m)
{
        ROS_INFO("Invalid Command 'drive' for go_to mode. Expected to receive a listen command first (typing '1')");
}

void State_GO_TO::listen(StateMachine* m)
{
        ROS_INFO("Switching to listening mode");
        m->change_state( new State_LISTENING() );
        delete this;
}

void State_GO_TO::go_to(StateMachine* m, string target)
{
        ROS_INFO("Invalid Command 'go to' for go_to mode. Expected to receive a listen command first (typing '1')");
}

void State_GO_TO::label(StateMachine* m, string label)
{
        ROS_INFO("Invalid Command 'label' for go_to mode. Expected to receive a listen command first (typing '1')");
}

void State_GO_TO::goal_reached(StateMachine* m)
{
        ROS_INFO("GOAL REACHED! Switching to listening mode");
        m->change_state( new State_LISTENING() );
        delete this;
}
