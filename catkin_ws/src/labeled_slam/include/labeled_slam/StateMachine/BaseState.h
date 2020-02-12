#ifndef BASE_STATE_H
#define BASE_STATE_H

#include "ros/ros.h"
#include <string>
using namespace std;
class StateMachine;

/** @class BaseState
 *  @brief Base class for all the specific state classes. Contains public interface of all derived classes.
 **/
class BaseState
{
public:
virtual void drive(StateMachine* m) = 0;
virtual void listen(StateMachine* m) = 0;
virtual void go_to(StateMachine* m, string target) = 0;
virtual void label(StateMachine* m, string label) = 0;
virtual void goal_reached(StateMachine* m) = 0;
};

#endif // BASE_STATE_H
