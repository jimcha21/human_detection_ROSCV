#include "ros/ros.h"
#include "std_msgs/String.h"
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

#include <stdlib.h>

#include <iostream>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/core/version.hpp>
#include <opencv2/contrib/contrib.hpp>

#include <libsgm.h>

using namespace cv;
using namespace std;

cv::Mat left_,right_;
bool img_received=false;
int done_=0;


int refresh_disp(){
	
	if(left_.rows==0 or right_.rows==0) return -1;	

	int disp_size = 128;
	int bits = 0;
	//cv::Mat left2_ = cv::imread("/home/ubuntu/Downloads/2014_imgleft/test_1/imgleft/imgleft000000299.pgm", -1);
	//cv::Mat right2_ = cv::imread("/home/ubuntu/Downloads/2014_imgright/test_1/imgright/imgright000000299.pgm", -1);

	//for debugging..
	//if(done_>1000) return -1;
	//done_++;


	switch (left_.type()) {
	case CV_8UC1: bits = 8; break;
	case CV_16UC1: bits = 16; break;
	default:
		std::cerr << "invalid input image color format" << left_.type() << std::endl;
		std::exit(EXIT_FAILURE);
	}

	sgm::StereoSGM ssgm(left_.cols, left_.rows, disp_size, bits, 8, sgm::EXECUTE_INOUT_HOST2HOST);

	cv::Mat output(cv::Size(left_.cols, left_.rows), CV_8UC1);

	ssgm.execute(left_.data, right_.data, (void**)&output.data);
// show image
	cv::imshow("image", output * 256 / disp_size);
	cv::waitKey(3);
}

void leftIMG_update(const sensor_msgs::ImageConstPtr& msg)
{
	cv_bridge::CvImagePtr cv_ptr;
	try
	{
		cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::MONO16);
	}
	catch (cv_bridge::Exception& e)
	{
		ROS_ERROR("cv_bridge exception: %s", e.what());
		return;
	}

	if(!img_received) img_received=true;	
	left_=cv_ptr->image.clone();

	cv::imshow("left_image", left_);
	cv::waitKey(3);

}

void rightIMG_update(const sensor_msgs::ImageConstPtr& msg)
{
	//ROS_INFO("HEE");
	cv_bridge::CvImagePtr cv_ptr;
	try
	{
		cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::MONO16);
	}
	catch (cv_bridge::Exception& e)
	{
		ROS_ERROR("cv_bridge exception: %s", e.what());
		return;
	}
	
	if(!img_received) img_received=true;
	refresh_disp();
	right_=cv_ptr->image.clone();

	cv::imshow("right_image", right_);
	cv::waitKey(3);

}

int main(int argc, char **argv)
{

	std::string rightCam_topic_name_ = "/guidance/right"; 
	std::string	leftCam_topic_name_  = "/guidance/left";
	if(argc!=3){
		cout<<"image_subb rightCam_topic_name leftCam_topic_name\n"<<endl;
		return -1;
	}
		
	ros::init(argc, argv, "stereo_oncameras_");
	ros::NodeHandle n;

	rightCam_topic_name_=argv[1];
	leftCam_topic_name_=argv[2];
	ROS_INFO("Subscribing on %s %s camera topics.",rightCam_topic_name_.c_str(),leftCam_topic_name_.c_str());

	ros::Subscriber sub_l = n.subscribe(leftCam_topic_name_, 1, leftIMG_update);
	ros::Subscriber sub_r = n.subscribe(rightCam_topic_name_, 1, rightIMG_update);   
	ros::spin();

	return 0;
}
