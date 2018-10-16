#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "geometry_msgs/Point.h"
#include "stdio.h"
#include <tf/transform_listener.h>
#include "math.h"

#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/PointCloud.h>

#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/conversions.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <sensor_msgs/PointCloud.h>

bool got_laser = false;
typedef pcl::PointCloud<pcl::PointXYZ> Pointcloud;
Pointcloud::Ptr output (new Pointcloud);
ros::Publisher pub;

void scanCallback (const sensor_msgs::LaserScan::ConstPtr& scan_in)
{
	output->header.stamp = pcl_conversions::toPCL( scan_in->header.stamp);
	output->header.frame_id = scan_in->header.frame_id;
	output->width  = 1; output->height = scan_in->ranges.size();
	output->is_dense = false;
	output->points.resize (output->width * output->height);
	ROS_INFO("elave scan");
	
	
	for(int i=0;i<scan_in->ranges.size();i++){
		
		tf::Point p_;
    p_.setX(scan_in->ranges[i]*cos( scan_in->angle_increment*i+ scan_in->angle_min)); 
    p_.setY(scan_in->ranges[i]*sin( scan_in->angle_increment*i+ scan_in->angle_min)); 
    p_.setZ(0); 

		output->points[i].x = p_.getX();
		output->points[i].y = p_.getY();
		output->points[i].z = p_.getZ();
			
	}

	pub.publish(output);
}

int main(int argc, char **argv)
{

  ros::init(argc, argv, "laserscan_toNao");

  ros::NodeHandle n;
  ros::Subscriber sub = n.subscribe("/scan", 1, scanCallback);
  pub = n.advertise<Pointcloud>("/pcl_scan", 1);
	//prob wants while ros ok and publish in
  //if(got_laser)
	ros::spin();
  return 0;

}