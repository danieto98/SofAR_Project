#include "StateMachine.h"
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "labeled_slam/Command.h"

#include <sstream>


void chatterCallback(const labeled_slam::Command::ConstPtr& msg)
{
        ROS_INFO("I heard: [%s]", msg->command.c_str());
}

int main(int argc, char **argv)
{
        /**
         * The ros::init() function needs to see argc and argv so that it can perform
         * any ROS arguments and name remapping that were provided at the command line.
         * For programmatic remappings you can use a different version of init() which takes
         * remappings directly, but for most command-line programs, passing argc and argv is
         * the easiest way to do it.  The third argument to init() is the name of the node.
         *
         * You must call one of the versions of ros::init() before using any other
         * part of the ROS system.
         */
        ros::init(argc, argv, "logic_node");
        /**
         * NodeHandle is the main access point to communications with the ROS system.
         * The first NodeHandle constructed will fully initialize this node, and the last
         * NodeHandle destructed will close down the node.
         */

        StateMachine state_machine;
        ros::NodeHandle n;
        /**
         * The advertise() function is how you tell ROS that you want to
         * publish on a given topic name. This invokes a call to the ROS
         * master node, which keeps a registry of who is publishing and who
         * is subscribing. After this advertise() call is made, the master
         * node will notify anyone who is trying to subscribe to this topic name,
         * and they will in turn negotiate a peer-to-peer connection with this
         * node.  advertise() returns a Publisher object which allows you to
         * publish messages on that topic through a call to publish().  Once
         * all copies of the returned Publisher object are destroyed, the topic
         * will be automatically unadvertised.
         *
         * The second parameter to advertise() is the size of the message queue
         * used for publishing messages.  If messages are published more quickly
         * than we can send them, the number here specifies how many messages to
         * buffer up before throwing some away.
         */
        ros::Publisher chatter_pub = n.advertise<labeled_slam::Command>("chatter", 1000);

        /*
           std_msgs::String msg;
           std::stringstream ss;
           ss << "hello world " << count;
           msg.data = ss.str();
           ROS_INFO("%s", msg.data.c_str());
           chatter_pub.publish(msg);
         */


        ros::Subscriber sub = n.subscribe("chatter", 1000, chatterCallback);

        ros::spin();

        return 0;
}
