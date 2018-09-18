#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "stdio.h"
#include <tf/transform_listener.h>


sensor_msgs::LaserScan laserscan_;

/*std_msgs/Header header
  uint32 seq
  time stamp
  string frame_id
  float32 angle_min
  float32 angle_max
  float32 angle_increment
  float32 time_increment
  float32 scan_time
  float32 range_min
  float32 range_max
  float32[] ranges
  float32[] intensities
*/

void laserCallback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
  laserscan_ = *msg;
  std::cout << laserscan_.header.frame_id << std::endl;
}

int main(int argc, char **argv)
{

  ros::init(argc, argv, "laserscan_toNao");

  ros::NodeHandle n;
  ros::Subscriber sub = n.subscribe("/scan", 1, laserCallback);


  tf::TransformListener listener;
  ros::Rate l(40);
  while(ros::ok){

    tf::StampedTransform transform;
    try{
      listener.lookupTransform("/map", "/laser", ros::Time(0), transform);
      listener.lookupTransform("/map", "/guidanceDown_leftcamera_opticalframe", ros::Time(0), transform);
    }
    catch (tf::TransformException ex){
      ROS_ERROR("%s",ex.what());
      ros::Duration(1.0).sleep();
    }


    l.sleep();
    ros::spinOnce();

  }

  return 0;
}
