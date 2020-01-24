#include "StateMachine.h"

StateMachine::StateMachine()
{
}


StateMachine::~StateMachine()
{
} // end ~NodeExample()



void StateMachine::callback(const labeled_slam::Command::ConstPtr& msg)
{
        string command = msg->command;
        string argument = msg->argument;


        // Note that these are only set to INFO so they will print to a terminal for example purposes.
        // Typically, they should be DEBUG.
        ROS_INFO("command is %s", command.c_str());
}    // end publishCallback()

void StateMachine::change_state(BaseState * state)
{
        state_ = state;
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
        ROS_INFO("Invalid Command GO_TO for driving mode. First say listen to go to listening mode.");
}

void State_DRIVING::label(StateMachine* m, string label)
{
        ROS_INFO("Invalid Command LABEL for driving mode. First say listen to go to listening mode.");
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
        ROS_INFO("Target: %s", target.c_str());
        m->change_state( new State_GO_TO(target) );
        delete this;
}

void State_LISTENING::label(StateMachine* m, string label)
{
        ROS_INFO("TODO!! Labeling");
}

State_GO_TO::State_GO_TO(string target)
        : target_(target)
{
}

void State_GO_TO::drive(StateMachine* m)
{
        ROS_INFO("Switching to driving mode");
        m->change_state( new State_DRIVING() );
        delete this;
}

void State_GO_TO::listen(StateMachine* m)
{
        ROS_INFO("Switching to listening mode");
        m->change_state( new State_LISTENING() );
        delete this;
}

void State_GO_TO::go_to(StateMachine* m, string target)
{
        ROS_INFO("Already in go_to mode!");
}

void State_GO_TO::label(StateMachine* m, string label)
{
        ROS_INFO("Invalid Command LABEL for go_to mode. First say listen to go to listening mode.");
}
