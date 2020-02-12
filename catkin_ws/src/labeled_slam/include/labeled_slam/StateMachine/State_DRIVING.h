#ifndef STATE_DRIVING_H
#define STATE_DRIVING_H

#include "BaseState.h"
class StateMachine;

/** @class State_DRIVING
 *  @brief State class for the mode, when the robot is controlled manually using the smartwatch
 **/
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

#endif //STATE_DRIVING_H
