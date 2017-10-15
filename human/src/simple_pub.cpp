#include "ros/ros.h"
#include "std_msgs/Bool.h"

#include <sstream>

std_msgs::Bool hello;

/**
 * This tutorial demonstrates simple sending of messages over the ROS system.
 */
int main(int argc, char **argv)
{

  ros::init(argc, argv, "talker");

 
  ros::NodeHandle n;

  ros::Publisher pub = n.advertise<std_msgs::Bool>("chatter", 1);

  ros::Rate loop_rate(10);


  int count = 0;
  while (ros::ok())
  {
 
    hello.data=true;
    pub.publish(hello);

    ros::spinOnce();

    loop_rate.sleep();
  }


  return 0;
}
