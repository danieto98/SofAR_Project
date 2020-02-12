#ifndef STATE_LISTENING_H
#define STATE_LISTENING_H

#include "BaseState.h"
class StateMachine;

/** @class State_LISTENING
 *  @brief State class for the mode, when the system is listening to voice commands.
 *
 *  Robot is not moving in this mode!
 **/
class State_LISTENING : public BaseState
{
public:
virtual void drive(StateMachine* m);
virtual void listen(StateMachine* m);
virtual void go_to(StateMachine* m, string target);
virtual void label(StateMachine* m, string label);
virtual void goal_reached(StateMachine* m);
};

#endif // STATE_LISTENING_H
