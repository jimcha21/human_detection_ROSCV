#include <iostream>
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <sensor_msgs/Temperature.h>
#include <sensor_msgs/Range.h>

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/highgui/highgui.hpp>

using namespace cv;
using namespace std;

struct tf_info {
  tf::Vector3 position;
  tf::Vector3 orientation;
};

//maybe import the below info from yaml..
//positions in reference with base_link
float guidancesensor_positions[5][3] = {   {0,0,-0.04995},
                                  {4,5,6},
                                  {4,5,6},
                                  {4,5,6},
                                  {7,8,9}  };

tf_info leftCamera_pose,rightCamera_pose,sonar_pose;
																	
void poseCallback(const ros::TimerEvent& event){
  cout<<"posting tf info"<<endl;
	left_camera.position=tf::Vector3(1,2,3);
	cout<<leftCamera.position.getX()<<endl;
/*  static tf::TransformBroadcaster br;
  tf::Transform transform;
  //cout<<transform.getBasis().getRow(0).getX()<<endl;
  transform.setOrigin( tf::Vector3( cameras_positions[0][0], cameras_positions[0][1], cameras_positions[0][2]) );
  tf::Quaternion q;
  q.setRPY(0, 0, 0);
  transform.setRotation(q);
  br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "base_link", "guidancedown_link"));*/
  return;
}

int main(int argc, char** argv){
  ros::init(argc, argv, "my_tf_broadcaster");
  
  cout<<guidancesensor_positions[0][0]<<endl;
  // 149870000.000
  // 39920000.000

  ros::NodeHandle node;
  //ros::Subscriber sub = node.subscribe(turtle_name+"/pose", 10, &poseCallback);

	ros::Rate loop_rate(1);
	while(ros::ok())
	{
		ros::Timer timer1 = node.createTimer(ros::Duration(1), poseCallback);
		ros::spin();
	}

  return 0;
};