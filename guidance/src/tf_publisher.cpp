#include <iostream>
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <sensor_msgs/Temperature.h>
#include <sensor_msgs/Range.h>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/highgui/highgui.hpp>


#define SONAR_TF 0
#define CAMERA_TF 1

using namespace cv;
using namespace std;

double _p =0.008325; // units in rviz which is 1cm in real world
double offset_z =-0.027;
double frontSensordist_x = 7*_p;
double downSensordist_z = -1.46*_p + offset_z;
double rearSensordist_x = -10.8*_p;
double rightSensordist_y = -8.5*_p;
double leftSensordist_y = -rightSensordist_y;
double camerafromcenter = 7.53*_p;	

cv_bridge::CvImage images[10];
sensor_msgs::Range ranges[5];
	

struct tf_info {
  tf::Vector3 position;
  tf::Vector3 rotation;
};

//maybe import the below info from yaml..
//positions in reference with base_link
double guidancesensor_positions[5][3] = {{0,0,downSensordist_z},
                                  {frontSensordist_x,0,offset_z},
                                  {-0.008275,rightSensordist_y,offset_z},
                                  {rearSensordist_x,0,offset_z},
                                  {-0.008275,leftSensordist_y,offset_z}};
																																														
double guidancesensor_rotations[5][3] = {{0,1.57,0},
                                  {0,0,0},
                                  {-1.57,0,0},
                                  {3.14,0,0},
                                  {1.57,0,0}};

tf_info leftCamera_pose,rightCamera_pose,sonar_pose;

string _whichSensorIsThis(int _id,int _type){
  int tmp_id_=_id;
  //if(_type==CAMERA_TF)
  //  tmp_id_=_id/2;

  if(tmp_id_==0)
		return string("Down");
	else if(tmp_id_==1)
		return string("Front");
	else if(tmp_id_==2)
		return string("Right");
	else if(tmp_id_==3)
		return string("Rear");
	else // if(tmp_id_==4)
		return string("Left");	
}

																	
bool publish_tf_(int sensor_location_,int sensor_type_){
	
	static tf::TransformBroadcaster br;
  tf::Transform transform;
  tf::Quaternion q;	
	string _parentTf = string("guidance")+_whichSensorIsThis(sensor_location_,sensor_type_)+string("_link");
	
	//posting guidance sensor tf
	transform.setOrigin(tf::Vector3(guidancesensor_positions[sensor_location_][0],guidancesensor_positions[sensor_location_][1],guidancesensor_positions[sensor_location_][2]));
	q.setRPY(guidancesensor_rotations[sensor_location_][2],guidancesensor_rotations[sensor_location_][1],guidancesensor_rotations[sensor_location_][0]);
	q.normalize();
	transform.setRotation(q);
	//base_link frame is above all M100 sensors..
	br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "base_link", _parentTf));	
	
	//and then the child tf
	switch(sensor_type_){
		case SONAR_TF: {
				//posting tf for sonar sensor
				transform.setOrigin(tf::Vector3(sonar_pose.position));
				q.setRPY(sonar_pose.rotation.getX(),sonar_pose.rotation.getY(),sonar_pose.rotation.getZ());
				q.normalize();
				transform.setRotation(q);  
				br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), _parentTf, string(ranges[sensor_location_].header.frame_id)));
			break;
		}
		case CAMERA_TF: {	 
				
			  string opticalframe_name_ = string("guidance")+_whichSensorIsThis(sensor_location_,sensor_type_); 
				
		   	//posting tf for the left camera
				transform.setOrigin(tf::Vector3(leftCamera_pose.position));
				q.setRPY(leftCamera_pose.rotation.getX(),leftCamera_pose.rotation.getY(),leftCamera_pose.rotation.getZ());
				q.normalize();
				transform.setRotation(q);
				br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), _parentTf, string(images[sensor_location_*2].header.frame_id)));
				
				//and its optical frame 
			  transform.setOrigin(tf::Vector3(0,0,0));
				q.setRPY(0,1.57,3.14);
				transform.setRotation(q);
				br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), string(images[sensor_location_*2].header.frame_id), opticalframe_name_+string("_leftcamera_opticalframe")));
			

				//and the right camera
				transform.setOrigin(tf::Vector3(rightCamera_pose.position));
				q.setRPY(rightCamera_pose.rotation.getX(),rightCamera_pose.rotation.getY(),rightCamera_pose.rotation.getZ());
				q.normalize();
				transform.setRotation(q);
				br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), _parentTf, string(images[(sensor_location_*2)+1].header.frame_id)));	
				
				//and its optical frame 
			  transform.setOrigin(tf::Vector3(0,0,0));
				q.setRPY(0,1.57,3.14);
				transform.setRotation(q);
				br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), string(images[(sensor_location_*2)+1].header.frame_id), opticalframe_name_+string("_rightcamera_opticalframe")));
			
				break;
			}			
	}

	return true;

}

void poseCallback(/*const ros::TimerEvent& event*/){
 
  cout<<"posting tf info"<<endl;
	for(int i=0;i<5;i++){
		publish_tf_(i,SONAR_TF);
		publish_tf_(i,CAMERA_TF);
	}	
    
}

int main(int argc, char** argv){

  ros::init(argc, argv, "my_tf_broadcastfer");
  //cout<<guidancesensor_positions[0][0]<<endl;
  // 149870000.000
  // 39920000.000
	
	
	leftCamera_pose.position=tf::Vector3(0.01,camerafromcenter,0); leftCamera_pose.rotation=tf::Vector3(0,0,0);
	rightCamera_pose.position=tf::Vector3(0.01,-camerafromcenter,0); rightCamera_pose.rotation=tf::Vector3(0,0,0);
	sonar_pose.position=tf::Vector3(0.01,0,0); sonar_pose.rotation=tf::Vector3(0,0,0);


images[0].header.frame_id = "guidanceDown_leftcamera_link";
images[1].header.frame_id = "guidanceDown_rightcamera_link";
images[2].header.frame_id = "guidanceFront_leftcamera_link";
images[3].header.frame_id = "guidanceFront_rightcamera_link";
images[4].header.frame_id = "guidanceRight_leftcamera_link";
images[5].header.frame_id = "guidanceRight_rightcamera_link";
images[6].header.frame_id = "guidanceRear_leftcamera_link";
images[7].header.frame_id = "guidanceRear_rightcamera_link";
images[8].header.frame_id = "guidanceLeft_leftcamera_link";
images[9].header.frame_id = "guidanceLeft_rightcamera_link";

ranges[0].header.frame_id = "ultrasonicDown_link";
ranges[1].header.frame_id = "ultrasonicFront_link";
ranges[2].header.frame_id = "ultrasonicRight_link";
ranges[3].header.frame_id = "ultrasonicRear_link";
ranges[4].header.frame_id = "ultrasonicLeft_link";


	
  ros::NodeHandle node;
  //ros::Subscriber sub = node.subscribe(turtle_name+"/pose", 10, &poseCallback);

  ros::Rate loop_rate(1);
  while(ros::ok())
  {
    //ros::Timer timer1 = node.createTimer(ros::Duration(1), poseCallback);
    
    poseCallback();
    ros::spinOnce();

    loop_rate.sleep();
  }

  return 0;
};

