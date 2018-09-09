#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
#include <stdio.h>

std::string marker_topic_="/ar_marker_1";

int main(int argc, char** argv){

  ros::init(argc, argv, "mapToM100_tflistener");
  ros::NodeHandle node;

  if(argc==2){
    marker_topic_=argv[1];
    std::cout<<"Setting armarker topic name as "<< marker_topic_<<std::endl;
  }
  else 
    std::cout<<"Setting armarker topic name as (default) "<< marker_topic_<<std::endl;

  ros::Publisher matrice_pose =
    node.advertise<geometry_msgs::TransformStamped>("/map_to_marker_tf", 10);

  tf::TransformListener listener;
  ros::Rate rate(10.0);

  while (node.ok()){
    tf::StampedTransform transform;
    try{
      listener.lookupTransform("/map", marker_topic_,
                               ros::Time(0), transform);
    }
    catch (tf::TransformException &ex) {
      ROS_ERROR("%s",ex.what());
      ros::Duration(1.0).sleep();
      continue;
    }

    geometry_msgs::TransformStamped transform_;
    transform_.header.stamp = ros::Time::now(); transform_.header.frame_id = "map";
    transform_.transform.translation.x = transform.getOrigin().x();
    transform_.transform.translation.y = transform.getOrigin().y();
    transform_.transform.translation.z = transform.getOrigin().z();

    transform_.transform.rotation.x = transform.getRotation().x();
    transform_.transform.rotation.y = transform.getRotation().y();
    transform_.transform.rotation.z = transform.getRotation().z();
    transform_.transform.rotation.w = transform.getRotation().w();

    matrice_pose.publish(transform_);

    rate.sleep();
  }
  return 0;
};