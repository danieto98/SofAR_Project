#include "StateMachine.h"
#include "ros/ros.h"
//
/**
 *  @brief This node implements the main logic of the labeled_slam project
 *
 *  The core object of this node is state_machine
 *  The c++ "state"-pattern is used to imlement the state machine in an object-
 *  oriented, safe and maintainable way
 *  Possible states of the state machine are "DRIVING", "LISTENING" and "GO_TO"
 **/
int main(int argc, char **argv)
{
        ros::init(argc, argv, "logic_node");

        ros::NodeHandle n;

        //Define all service clients
        ros::ServiceClient client_set_goal                = n.serviceClient<SRV_TYPE_SET_GOAL>("set_goal");
        ros::ServiceClient client_set_label               = n.serviceClient<SRV_TYPE_SET_LABEL>("set_label");
        ros::ServiceClient client_activate_path_following = n.serviceClient<std_srvs::SetBool>("activate_path_following");
        ros::ServiceClient client_activate_driving        = n.serviceClient<std_srvs::SetBool>("activate_driving");

        //Create state machine, pass all the service clients to state machine (services will be called from inside)
        StateMachine state_machine(&client_set_goal,
                                   &client_set_label,
                                   &client_activate_path_following,
                                   &client_activate_driving);

        //Create Subscribers, assign callback-functions to memberfunctions of state_machine
        ros::Subscriber sub_command       = n.subscribe("text_command", 1000, &StateMachine::callback_command, &state_machine);
        ros::Subscriber sub_goal_reached  = n.subscribe("goal_reached", 1000, &StateMachine::callback_goal_reached, &state_machine);

        ros::spin();

        return 0;
}
