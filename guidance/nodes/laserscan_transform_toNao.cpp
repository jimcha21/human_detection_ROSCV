#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "geometry_msgs/Point.h"
#include "stdio.h"
#include <tf/transform_listener.h>
#include "math.h"

sensor_msgs::LaserScan laserscan_,naolaser_;
bool got_laser = false;

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
  laserscan_ = naolaser_ = *msg;
  naolaser_.header.frame_id = "ar_marker_4";
  std::cout << laserscan_.ranges.size() << std::endl;
  got_laser=true;

}

int main(int argc, char **argv)
{

  ros::init(argc, argv, "laserscan_toNao");

  ros::NodeHandle n;
  ros::Subscriber sub = n.subscribe("/scan", 1, laserCallback);
  ros::Publisher pub = n.advertise<sensor_msgs::LaserScan>("nao_scan", 1);

  tf::TransformListener listener;
  ros::Rate l(40);

  while(ros::ok){

    if(!got_laser)
    {
      l.sleep();
      ros::spinOnce();
      continue;
    }      

/*    tf::StampedTransform mapTolaser_tf,mapToarmarker_tf;
    try{
      listener.lookupTransform("/map", "/laser", ros::Time(0), mapTolaser_tf);
      listener.lookupTransform("/map", "/guidanceDown_leftcamera_opticalframe", ros::Time(0), mapToarmarker_tf);
    }
    catch (tf::TransformException ex){
      ROS_ERROR("%s",ex.what());
      ros::Duration(1.0).sleep();
    }*/



     pub.publish(naolaser_);

    l.sleep();
    ros::spinOnce();

  }

  return 0;
}