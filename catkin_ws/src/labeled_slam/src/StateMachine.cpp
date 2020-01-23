#include "StateMachine.h"
/*--------------------------------------------------------------------
 * NodeExample()
 * Constructor.
 *------------------------------------------------------------------*/

StateMachine::StateMachine()
{
} // end NodeExample()

/*--------------------------------------------------------------------
 * ~NodeExample()
 * Destructor.
 *------------------------------------------------------------------*/

StateMachine::~StateMachine()
{
} // end ~NodeExample()


/*--------------------------------------------------------------------
 * messageCallback()
 * Callback function for subscriber.
 *------------------------------------------------------------------*/

/*
   void state_machine::messageCallback(const node_example::node_example_data::ConstPtr &msg)
   {
        message = msg->message;
        a = msg->a;
        b = msg->b;

        // Note that these are only set to INFO so they will print to a terminal for example purposes.
        // Typically, they should be DEBUG.
        ROS_INFO("message is %s", message.c_str());
        ROS_INFO("sum of a + b = %d", a + b);
   } // end publishCallback()
 */
