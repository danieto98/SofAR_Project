#ifndef STATE_GO_TO_H
#define STATE_GO_TO_H

#include "BaseState.h"
class StateMachine;

/** @class State_GO_TO
 *  @brief State class for the mode, when the system is following a path towards a target.
 *
 *  Target can be one of the labels which have been created before
 **/
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

#endif // STATE_GO_TO_H
