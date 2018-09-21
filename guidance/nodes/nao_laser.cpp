#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "stdio.h"

bool got_laser = false;
sensor_msgs::LaserScan laser_scan_;
ros::Publisher pub;

void scanCallback (const sensor_msgs::LaserScan::ConstPtr& scan_in)
{
	laser_scan_ = *scan_in;
	laser_scan_.header.frame_id = "base_link";
	pub.publish(laser_scan_);
}

int main(int argc, char **argv)
{

  ros::init(argc, argv, "nao_laser");

  ros::NodeHandle n;
  ros::Subscriber sub = n.subscribe("/nao_scan", 1, scanCallback);
  pub = n.advertise<sensor_msgs::LaserScan>("/nao_baselink_scan", 1);
	//prob wants while ros ok and publish in
  //if(got_laser)
	ros::spin();
  return 0;

}