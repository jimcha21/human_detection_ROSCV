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

double _p =0.008325;

struct tf_info {
  tf::Vector3 position;
  tf::Vector3 rotation;
};

//maybe import the below info from yaml..
//positions in reference with base_link
double guidancesensor_positions[5][3] = {{0,0,-0.044},
                                  {0.06,0,-0.027},
                                  {-0.008275,0.08,-0.027},
                                  {-0.0898,0,-0.027},
                                  {-0.008275,-0.0698,-0.027}};

double guidancesensor_rotations[5][3] = {{1.57,0,-1.57},
                                  {-1.570,0,0},
                                  {0,0,0},
                                  {1.570,0,0},
                                  {3.141,0,0}};

tf_info leftCamera_pose,rightCamera_pose,sonar_pose;

bool postGuidanceSensorChilds_tfs(string _parentTf){
  static tf::TransformBroadcaster br;
  tf::Transform transform;
  tf::Quaternion q;

  transform.setOrigin(tf::Vector3(leftCamera_pose.position));
  q.setRPY(leftCamera_pose.rotation.getX(),leftCamera_pose.rotation.getY(),leftCamera_pose.rotation.getZ());
  q.normalize();
  transform.setRotation(q);
  br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), _parentTf, "guidanceDown_leftcamera_link"));

  transform.setOrigin(tf::Vector3(rightCamera_pose.position));
  q.setRPY(rightCamera_pose.rotation.getX(),rightCamera_pose.rotation.getY(),rightCamera_pose.rotation.getZ());
  q.normalize();
  transform.setRotation(q);
  br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), _parentTf, "guidanceDown_rightcamera_link"));

  transform.setOrigin(tf::Vector3(sonar_pose.position));
  q.setRPY(sonar_pose.rotation.getX(),sonar_pose.rotation.getY(),sonar_pose.rotation.getZ());
  q.normalize();
  transform.setRotation(q);  
  br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), _parentTf, "ultrasonicDown_link"));
}   

bool postGuidanceSensorChilds_tfs2(string _parentTf){
  static tf::TransformBroadcaster br;
  tf::Transform transform;
  tf::Quaternion q;

  transform.setOrigin(tf::Vector3(leftCamera_pose.position));
  q.setRPY(leftCamera_pose.rotation.getX(),leftCamera_pose.rotation.getY(),leftCamera_pose.rotation.getZ());
  q.normalize();
  transform.setRotation(q);
  br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), _parentTf, "guidanceFront_leftcamera_link"));

  transform.setOrigin(tf::Vector3(rightCamera_pose.position));
  q.setRPY(rightCamera_pose.rotation.getX(),rightCamera_pose.rotation.getY(),rightCamera_pose.rotation.getZ());
  q.normalize();
  transform.setRotation(q);
  br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), _parentTf, "guidanceFront_rightcamera_link"));

  transform.setOrigin(tf::Vector3(sonar_pose.position));
  q.setRPY(sonar_pose.rotation.getX(),sonar_pose.rotation.getY(),sonar_pose.rotation.getZ());
  q.normalize();
  transform.setRotation(q);  
  br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), _parentTf, "ultrasonicFront_link"));

}   


void poseCallback3(const ros::TimerEvent& event){
	cout<<"geia"<<endl;
	
	}

	void poseCallback4(const ros::TimerEvent& event){
	cout<<"geia4"<<endl;
	
	}
	
void poseCallback(/*const ros::TimerEvent& event*/){
 
  cout<<"posting tf info"<<endl;
 /* cout<<leftCamera.position.getX()<<endl;
*/
  static tf::TransformBroadcaster br;
  tf::Transform transform;
  tf::Quaternion q;

  transform.setOrigin(tf::Vector3(guidancesensor_positions[0][0],guidancesensor_positions[0][1],guidancesensor_positions[0][2]));
  q.setRPY(guidancesensor_rotations[0][0],guidancesensor_rotations[0][1],guidancesensor_rotations[0][2]);
  transform.setRotation(q);
  br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "base_link", "guidanceDown_link"));
  postGuidanceSensorChilds_tfs("guidanceDown_link");

  transform.setOrigin(tf::Vector3(guidancesensor_positions[1][0],guidancesensor_positions[1][1],guidancesensor_positions[1][2]));
  q.setRPY(guidancesensor_rotations[1][0],guidancesensor_rotations[1][1],guidancesensor_rotations[1][2]);
  transform.setRotation(q);
  br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "base_link", "guidanceFront_link"));
  postGuidanceSensorChilds_tfs("guidanceFront_link");
    
}

int main(int argc, char** argv){

  ros::init(argc, argv, "my_tf_broadcaster");
  //cout<<guidancesensor_positions[0][0]<<endl;
  // 149870000.000
  // 39920000.000

leftCamera_pose.position=tf::Vector3(7.33*_p,0.01,0); leftCamera_pose.rotation=tf::Vector3(0,0,0);
rightCamera_pose.position=tf::Vector3(-7.33*_p,0.01,0); rightCamera_pose.rotation=tf::Vector3(0,0,0);
sonar_pose.position=tf::Vector3(0,0.01,0); sonar_pose.rotation=tf::Vector3(0,0,0);


  ros::NodeHandle node;
  //ros::Subscriber sub = node.subscribe(turtle_name+"/pose", 10, &poseCallback);

  ros::Rate loop_rate(1);
	
		ros::Timer timer1 = node.createTimer(ros::Duration(1), poseCallback3);
		ros::Timer timer13 = node.createTimer(ros::Duration(2), poseCallback4);
  while(ros::ok())
  {    
    //poseCallback();
		cout<<"me lene kosnatantino"<<endl;
    ros::spinOnce();

    loop_rate.sleep();
  }

  return 0;
};

