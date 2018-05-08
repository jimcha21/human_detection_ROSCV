#include "ros/ros.h"
#include "sensor_msgs/CameraInfo.h"

#include <iostream>
#include <sstream>

using namespace std;

int main(int argc, char **argv)
{
  ros::init(argc, argv, "talker");

  ros::NodeHandle n;
  ros::Rate loop_rate(10);

	ros::Publisher pub_d_l = n.advertise<sensor_msgs::CameraInfo >("/guidance/down/left_camera/camera_info", 1);
	ros::Publisher pub_d_r = n.advertise<sensor_msgs::CameraInfo >("/guidance/down/right_camera/camera_info", 1);
	ros::Publisher pub_f_l = n.advertise<sensor_msgs::CameraInfo >("/guidance/front/left_camera/camera_info", 1);
  ros::Publisher pub_f_r = n.advertise<sensor_msgs::CameraInfo >("/guidance/front/right_camera/camera_info", 1);
  ros::Publisher pub_g_l = n.advertise<sensor_msgs::CameraInfo >("/guidance/right/left_camera/camera_info", 1);
  ros::Publisher pub_g_r = n.advertise<sensor_msgs::CameraInfo >("/guidance/right/right_camera/camera_info", 1);	
  ros::Publisher pub_r_l = n.advertise<sensor_msgs::CameraInfo >("/guidance/rear/left_camera/camera_info", 1);
  ros::Publisher pub_r_r = n.advertise<sensor_msgs::CameraInfo >("/guidance/rear/right_camera/camera_info", 1);
  ros::Publisher pub_l_l = n.advertise<sensor_msgs::CameraInfo >("/guidance/left/left_camera/camera_info", 1);
  ros::Publisher pub_l_r = n.advertise<sensor_msgs::CameraInfo >("/guidance/left/right_camera/camera_info", 1);
	
	//initializgf cameras info -  todo make it read from yaml file intriextri
	sensor_msgs::CameraInfo leftCamera_info_,rightCamera_info_;

	leftCamera_info_.height = rightCamera_info_.height = 240;
	leftCamera_info_.width = rightCamera_info_.width = 320;
	leftCamera_info_.distortion_model = rightCamera_info_.distortion_model = "plumb_bob";

	leftCamera_info_.D.push_back(-0.001980888066537227);
	leftCamera_info_.D.push_back(-0.02147700025762941);
	leftCamera_info_.D.push_back( 0.0006841015603197759);
	leftCamera_info_.D.push_back(-0.00108815737201456);
	leftCamera_info_.D.push_back(0);
	leftCamera_info_.K = {247.35757622530616, 0.0, 153.29506251287017, 0.0, 247.39002468328675, 116.8939252668646, 0.0, 0.0, 1.0};
	leftCamera_info_.R = {0.9995417232106595, 0.0032279128371214463, -0.0300985737167282, -0.0032132327445043023, 0.9999946938578694, 0.0005360890183786829, 0.03010014445804239, -0.0004391296185950722, 0.9995467915354354};
	leftCamera_info_.P = {251.2699111385827, 0.0, 163.1802978515625, 0.0, 0.0, 251.2699111385827, 116.94859886169434, 0.0, 0.0, 0.0, 1.0, 0.0}     ;

	rightCamera_info_.D.push_back(-0.00823560228072968);
	rightCamera_info_.D.push_back(0.011941631523956353);
	rightCamera_info_.D.push_back(-0.0008137087854255292);
	rightCamera_info_.D.push_back(0.005684827477769688);
	rightCamera_info_.D.push_back(0);
	
	rightCamera_info_.K = {248.97894170631278, 0.0, 161.85038053113018, 0.0, 248.68035901590156, 117.07950980602632, 0.0, 0.0, 1.0};
	rightCamera_info_.R = {0.9999917289986313, 0.0035033775229645864, 0.002065981669714303, -0.003502369596947691, 0.9999937460039977, -0.0004912841321297013, -0.0020676899028590923, 0.0004840442373301131, 0.9999977451772789};
	rightCamera_info_.P = {251.2699111385827, 0.0, 163.1802978515625, -117.04901478227022, 0.0, 251.2699111385827, 116.94859886169434, 0.0, 0.0, 0.0, 1.0, 0.0};

  while (ros::ok())
  {
		cout<<"posting"<<endl;
		
		leftCamera_info_.header.frame_id="guidanceDown_leftcamera_opticalframe";
		pub_d_l.publish(leftCamera_info_);		
		leftCamera_info_.header.frame_id="guidanceFront_leftcamera_opticalframe";
		pub_f_l.publish(leftCamera_info_);		
		leftCamera_info_.header.frame_id="guidanceRight_leftcamera_opticalframe";
		pub_g_l.publish(leftCamera_info_);		
		leftCamera_info_.header.frame_id="guidanceRear_leftcamera_opticalframe";
		pub_r_l.publish(leftCamera_info_);		
		leftCamera_info_.header.frame_id="guidanceLeft_leftcamera_opticalframe";
		pub_l_l.publish(leftCamera_info_);		
		
		rightCamera_info_.header.frame_id="guidanceDown_rightcamera_opticalframe";
		pub_d_r.publish(rightCamera_info_);
		rightCamera_info_.header.frame_id="guidanceFront_rightcamera_opticalframe";
		pub_f_r.publish(rightCamera_info_);
		rightCamera_info_.header.frame_id="guidanceLeft_rightcamera_opticalframe";
		pub_l_r.publish(rightCamera_info_);
		rightCamera_info_.header.frame_id="guidanceRear_rightcamera_opticalframe";
		pub_r_r.publish(rightCamera_info_);
		rightCamera_info_.header.frame_id="guidanceRight_rightcamera_opticalframe";
		pub_g_r.publish(rightCamera_info_);		
		
    ros::spinOnce();
    loop_rate.sleep();

	}

	leftCamera_info_.D.clear();
	rightCamera_info_.D.clear();
  return 0;

}