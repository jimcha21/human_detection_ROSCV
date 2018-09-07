
#include "ros/ros.h"
#include <stdio.h>
#include <std_msgs/Bool.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <ar_track_alvar_msgs/AlvarMarker.h>
#include <ar_track_alvar_msgs/AlvarMarkers.h>
#include <tf/transform_broadcaster.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

ar_track_alvar_msgs::AlvarMarker marker_;
geometry_msgs::TransformStamped map_to_marker_tf;
geometry_msgs::PoseStamped nao_pose_to_map_coord_;
bool isMaponline = false;

void map_to_marker_tfcallback(const geometry_msgs::TransformStamped& msg)
{
  map_to_marker_tf = msg ;
  //std::cout<< map_to_marker_tf<<std::endl;
  isMaponline = true;
}

void markerCallback(const ar_track_alvar_msgs::AlvarMarkersPtr& msg)
{
  if(msg->markers.size()==0&&isMaponline)
    return;


  //std::cout<<"aaaa\n"<< *msg<<"aaaa\n"<<std::endl;

  static tf2_ros::TransformBroadcaster br;
  geometry_msgs::TransformStamped map_to_odom_tf;
  
  map_to_odom_tf.header.stamp = map_to_marker_tf.header.stamp;
  map_to_odom_tf.header.frame_id = "map";
  map_to_odom_tf.child_frame_id = "odom";
  
	tf::vector3TFToMsg(tf::Vector3(map_to_marker_tf.transform.translation.x,map_to_marker_tf.transform.translation.y,0),map_to_odom_tf.transform.translation); 
  tf::Quaternion q;
	tf::quaternionMsgToTF(map_to_marker_tf.transform.rotation,q);
	tf::Matrix3x3 m(q);
	tfScalar y,p,r;
	m.getEulerYPR(y,p,r);
	q = tf::createQuaternionFromRPY(0,0,y);
	q.normalize();
	tf::quaternionTFToMsg(q,map_to_odom_tf.transform.rotation);
	
  tf2::doTransform(msg->markers[0].pose, nao_pose_to_map_coord_, map_to_odom_tf);
  //updating tf tree for nao_pose under map coordinates..
  br.sendTransform(map_to_odom_tf);

  //std::cout<<"got marker "<<nao_pose_to_map_coord_.pose.position.x<<"got marker "<<nao_pose_to_map_coord_.pose.position.y<<"got marker "<<nao_pose_to_map_coord_.pose.position.z<<std::endl;
   
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "nao_locator");
  ros::NodeHandle n;

  ros::Publisher initial_naopose_pub = n.advertise<geometry_msgs::PoseWithCovarianceStamped>("/initialpose", 1);
  ros::Subscriber mark_sub = n.subscribe("/ar_pose_marker", 1, &markerCallback);
  ros::Subscriber tf_sub = n.subscribe("/map_to_marker_tf", 1, &map_to_marker_tfcallback);

  ros::Rate rate(1.0);

  while(n.ok()){

    geometry_msgs::PoseWithCovarianceStamped a;
    a.header = nao_pose_to_map_coord_.header;
    a.pose.pose = nao_pose_to_map_coord_.pose;
    a.pose.pose.position.z = 0;
    for (int i = 0; i < 36; ++i)
      a.pose.covariance[i] = 0.0f;
    a.pose.covariance[0] = a.pose.covariance[7] = 0.25f;
    a.pose.covariance[35] = 0.06853891945200942;

    //keep publishing until ros::kill .. fake_localization node subs on it ..
    initial_naopose_pub.publish(a);
    //return 0;
    //std::cout<<"publishing initial pose"<<std::endl;
    rate.sleep();
    ros::spinOnce();  
      
  }
  return 0;
}

 /*   std::cout<<"Marker pose in reference to from tf coord "<<transform.getRotation().x()<<" "<<transform.getRotation().y()<<" "<<transform.getRotation().z()<<" "<<transform.getRotation().w()<<std::endl;
    //2d map, ignoring z value 
    transform.setOrigin(tf::Vector3(transform.getRotation().x(),transform.getRotation().y(),0));
    //check for denormalized-zero quat
    if(transform.getRotation().x()==0 && transform.getRotation().y()==0 &&transform.getRotation().z()==0 &&transform.getRotation().w()==0)
    continue;
    br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "map", "base_link"));*/

