
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
bool naopose_legit = false;

void map_to_marker_tfcallback(const geometry_msgs::TransformStamped& msg)
{
  map_to_marker_tf = msg ;
  //std::cout<< map_to_marker_tf<<std::endl;
  isMaponline = true;
}

void markerCallback(const ar_track_alvar_msgs::AlvarMarkersPtr& msg)
{
  if(msg->markers.size()==0||!isMaponline)
    return;
	
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

  geometry_msgs::PoseStamped zero_pose_;
  zero_pose_.header.stamp = map_to_marker_tf.header.stamp;
  zero_pose_.header.frame_id = "map";
  tf::Quaternion qq= tf::createQuaternionFromRPY(0, 0, 0).normalize();
  tf::quaternionTFToMsg(qq,zero_pose_.pose.orientation);
  
  tf2::doTransform(zero_pose_, nao_pose_to_map_coord_, map_to_odom_tf);
  //tf2::doTransform(msg->markers[0].pose, nao_pose_to_map_coord_, map_to_odom_tf);
  
  //updating tf tree for nao_pose under map coordinates.. disable it if fake_loc node is running - conflict in tree tf publishing 
  br.sendTransform(map_to_odom_tf);
	naopose_legit = true;
  //std::cout<<"got marker "<<nao_pose_to_map_coord_.pose.position.x<<"got marker "<<nao_pose_to_map_coord_.pose.position.y<<"got marker "<<nao_pose_to_map_coord_.pose.position.z<<std::endl;
   
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "nao_locator");
  ros::NodeHandle n;

  ros::Publisher initial_naopose_pub = n.advertise<geometry_msgs::PoseWithCovarianceStamped>("/initialpose", 1);
  ros::Subscriber mark_sub = n.subscribe("/ar_pose_marker", 1, &markerCallback);
  ros::Subscriber tf_sub = n.subscribe("/map_to_marker_tf", 1, &map_to_marker_tfcallback);

  ros::Rate rate(10.0);

  while(n.ok()){

    // bypassing -nan values in nao pose initialization
    if(nao_pose_to_map_coord_.pose.position.x!=nao_pose_to_map_coord_.pose.position.x && !naopose_legit){
      rate.sleep();
      ros::spinOnce();  
      continue;
    }    

    geometry_msgs::PoseWithCovarianceStamped nao_initial_pose_;
    nao_initial_pose_.header = nao_pose_to_map_coord_.header;
    nao_initial_pose_.pose.pose = nao_pose_to_map_coord_.pose;
    //nao_initial_pose_.pose.pose.position.z = 0;
    for (int i = 0; i < 36; ++i) nao_initial_pose_.pose.covariance[i] = 0.0f;
    nao_initial_pose_.pose.covariance[0] = nao_initial_pose_.pose.covariance[7] = 0.25f;
    nao_initial_pose_.pose.covariance[35] = 0.06853891945200942;

    //keep publishing until ros::kill .. fake_localization node subs on it ..
    initial_naopose_pub.publish(nao_initial_pose_);
    //return 0;
    //std::cout<<"publishing initial pose"<<std::endl;
    rate.sleep();
    ros::spinOnce();       
  }
  return 0;
}
