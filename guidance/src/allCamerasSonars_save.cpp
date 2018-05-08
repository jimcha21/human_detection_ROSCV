/*
 * GuidanceNode.cpp
 *
 *  Created on: Apr 29, 2015
 */

#include <stdio.h>
#include <string.h>
#include <iostream>
#include <sstream>
#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>

#include <opencv2/opencv.hpp>

#include "DJI_guidance.h"
#include "DJI_utility.h"

#include <tf/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h> //IMU
#include <geometry_msgs/Vector3Stamped.h> //velocity
#include <sensor_msgs/LaserScan.h> //obstacle distance & ultrasonic
#include <sensor_msgs/Range.h>

#include <sys/statvfs.h>
#include <iostream>
#include <cstring>

#include <cstdlib> 

using namespace std;
using namespace cv;

#define SONAR_TF 0
#define CAMERA_TF 1
#define WIDTH 320
#define HEIGHT 240
#define IMAGE_SIZE (HEIGHT * WIDTH)

struct tf_info {
  tf::Vector3 position;
  tf::Vector3 rotation;
};

//scale parameter for transformin sizes from cm in rviz units 
double _p =0.008325; // units in rviz which is 1cm in real world
double offset_z =-0.027;
double frontSensordist_x = 7*_p;
double downSensordist_z = -1.46*_p + offset_z;
double rearSensordist_x = -10.8*_p;
double rightSensordist_y = -8.5*_p;
double leftSensordist_y = -rightSensordist_y;
double camerafromcenter = 7.53*_p;	

/*<arg name="cmtorviz" value="0.008325" />  <!-- 1cm -> 0.008325 units in rviz -->
<arg name="offset_z" value="-0.027" />
<arg name="frontSensordist_x" value="$(eval 7*arg('cmtorviz'))"/>
<arg name="downSensordist_z"  value="$(eval -1.46*arg('cmtorviz') + arg('offset_z'))"/>
<arg name="rearSensordist_x"  value="$(eval -10.8*arg('cmtorviz'))"/>
<arg name="rightSensordist_y" value="$(eval -8.5*arg('cmtorviz'))"/>
<arg name="leftSensordist_y"  value="$(eval 8.5*arg('cmtorviz'))"/>
*/

//positions in reference with base_link
/*double guidancesensor_positions[5][3] = {{0,0,-0.044},
                                  {0.06,0,-0.027},
                                  {-0.008275,0.08,-0.027},
                                  {-0.0898,0,-0.027},
                                  {-0.008275,-0.0698,-0.027}};*/

double guidancesensor_positions[5][3] = {{0,0,downSensordist_z},
                                  {frontSensordist_x,0,offset_z},
                                  {-0.008275,rightSensordist_y,offset_z},
                                  {rearSensordist_x,0,offset_z},
                                  {-0.008275,leftSensordist_y,offset_z}};
																
																	
//maybe import the below guidance sensors' info from a yaml file ~
double guidancesensor_rotations[5][3] = {{0,1.57,0},
                                  {0,0,0},
                                  {-1.57,0,0},
                                  {3.14,0,0},
                                  {1.57,0,0}};
								  
tf_info leftCamera_pose,rightCamera_pose,sonar_pose;	

//ROS params																	
ros::Publisher image_pubs[10]; 
ros::Publisher range_pubs[5];
ros::Publisher ultrasonic_pub;
cv_bridge::CvImage images[10];
sensor_msgs::Range ranges[5];

Mat greyscales[10] {Mat(HEIGHT, WIDTH, CV_8UC1),Mat(HEIGHT, WIDTH, CV_8UC1),Mat(HEIGHT, WIDTH, CV_8UC1),Mat(HEIGHT, WIDTH, CV_8UC1),Mat(HEIGHT, WIDTH, CV_8UC1),
Mat(HEIGHT, WIDTH, CV_8UC1),Mat(HEIGHT, WIDTH, CV_8UC1),Mat(HEIGHT, WIDTH, CV_8UC1),Mat(HEIGHT, WIDTH, CV_8UC1),Mat(HEIGHT, WIDTH, CV_8UC1)};

char       key         = 0;
bool       enable_sonars= false;
bool       enable_cams = false;
bool       store_images = false;
std::string    path = "/media/ubuntu/USB"; //default path

DJI_lock   g_lock;
DJI_event  g_event;

std::ostream& operator<<(std::ostream& out, const e_sdk_err_code value){
const char* s = 0;
   static char str[100]={0};
#define PROCESS_VAL(p) case(p): s = #p; break;
   switch(value){
    PROCESS_VAL(e_OK);     
    PROCESS_VAL(e_load_libusb_err);     
    PROCESS_VAL(e_sdk_not_inited);
    PROCESS_VAL(e_disparity_not_allowed);
    PROCESS_VAL(e_image_frequency_not_allowed);
    PROCESS_VAL(e_config_not_ready);
    PROCESS_VAL(e_online_flag_not_ready);
    PROCESS_VAL(e_stereo_cali_not_ready);
    PROCESS_VAL(e_libusb_io_err);
    PROCESS_VAL(e_timeout);
    default:
    strcpy(str, "Unknown error");
    s = str;
    break;
  }
#undef PROCESS_VAL
  return out << s;
}

//returns in string the sensor id
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


//Checking available space in $path directory.
bool storage_check()
{
    struct statvfs fiData;
    if(path.c_str() == NULL){
		//cerr << "Error..." << endl;
		return 0;
	}
    
    if((statvfs(path.c_str(),&fiData)) < 0 ) {
			cout << "\nFailed to stat:"  << endl;
			return false;
	} else {
		//cout << "\nDisk: " <<  argv[i];
/*			cout << "\nBlock size: "<< fiData.f_bsize;
			cout << "\nTotal no blocks: "<< fiData.f_blocks;
			cout << "\nFree blocks: "<< fiData.f_bfree;*/
			
			//if (fiData.f_bsize*fiData.f_bfree<3)
			float free_space = (long long)(fiData.f_bfree)*(long long)fiData.f_bsize * pow(10,-9);
			//cout << free_space in gigs << endl; 	
			if (free_space<0.1)	return false;
	}	
	return true;
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
				q.setRPY(0,-1.57,0);
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
				q.setRPY(0,-1.57,0);
				transform.setRotation(q);
				br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), string(images[(sensor_location_*2)+1].header.frame_id), opticalframe_name_+string("_rightcamera_opticalframe")));
			
				break;
		}			
	}
	
	return true;

}

void postTf_sch_(const ros::TimerEvent& event){
	cout<<"posting tf frames..."<<endl;
	for(int i=0;i<5;i++){
		publish_tf_(i,SONAR_TF);
		publish_tf_(i,CAMERA_TF);
	}	
}

int my_callback(int data_type, int data_len, char *content)
{
  g_lock.enter();

    /* image data */
  if (e_image == data_type && NULL != content && enable_cams)
  {        
    image_data* data = (image_data*)content;
    
    for (int i = 0; i < 5; ++i)
    {
			int j = 2*i;
			//publish_tf_(j,CAMERA_TF); // publish tf (sonar)
			
			if(data->m_greyscale_image_left[i]){
        memcpy(greyscales[j].data, data->m_greyscale_image_left[i], IMAGE_SIZE);
		    greyscales[j].copyTo(images[j].image);

        if(store_images && storage_check()){
            vector<int> compression_params;
            compression_params.push_back(CV_IMWRITE_PNG_COMPRESSION);
            compression_params.push_back(9);

            std::string name=path;//"/media/ubuntu/FLIR_DATA1/stereo_footage/left_cam_";
            std::stringstream id_info,id_info2;
            id_info << i+1;
            name=name + id_info.str();
            id_info2 << ros::Time::now().toNSec();
            //std::cout <<  ros::Time::now().toSec() <<std::endl;
            name=name + "_" + id_info2.str() +".png" ;
            imwrite(name, images[j].image, compression_params);
        }


        images[j].header.stamp  = ros::Time::now();
        images[j].encoding    = sensor_msgs::image_encodings::MONO8;
        image_pubs[j].publish(images[j].toImageMsg());
      }
   
      if(data->m_greyscale_image_right[i]){
        memcpy(greyscales[j+1].data, data->m_greyscale_image_right[i], IMAGE_SIZE);
		    greyscales[j+1].copyTo(images[j+1].image);

        if(store_images && storage_check()){
            vector<int> compression_params;
            compression_params.push_back(CV_IMWRITE_PNG_COMPRESSION);
            compression_params.push_back(9); //                                         0-9 the higher the value is the biggest the compression check with lower vallue 3:default )((*)(

            std::string name=path;//"/media/ubuntu/FLIR_DATA1/stereo_footage/right_cam_";
            std::stringstream id_info,id_info2;
            id_info << i+1;
            name=name + id_info.str();
            id_info2 << ros::Time::now().toNSec();
            //std::cout <<  ros::Time::now().toSec() <<std::endl;
            name=name + "_" + id_info2.str() +".png" ;
            imwrite(name, images[j+1].image, compression_params);
        }

        images[j+1].header.stamp = ros::Time::now();
        images[j+1].encoding = sensor_msgs::image_encodings::MONO8;
        image_pubs[j+1].publish(images[j+1].toImageMsg());
      }
    }

  }

      /* ultrasonic */
    if ( e_ultrasonic == data_type && NULL != content && enable_sonars)
    {
        ultrasonic_data *ultrasonic = (ultrasonic_data*)content;
        /*if (verbosity > 1) {
            printf( "frame index: %d, stamp: %d\n", ultrasonic->frame_index, ultrasonic->time_stamp );
            for ( int d = 0; d < 5; ++d )
            {
                printf( "ultrasonic distance: %f, reliability: %d\n", ultrasonic->ultrasonic[d] * 0.001f, (int)ultrasonic->reliability[d] );
            }
        }*/

        for ( int d = 0; d < 5; ++d ){
          // publish ultrasonic data
          ranges[d].header.stamp = ros::Time::now();
          ranges[d].radiation_type = 0; //ULTRASOUND=0 , INFRARED=1 
          ranges[d].field_of_view = 1; //CHeck dis also - check manual
          ranges[d].min_range = 0; ranges[d].max_range = 20; //not the correct values probably - PLEASE CHECK
          ranges[d].range = 0.001f * ultrasonic->ultrasonic[d];
          
					 // publish tf (sonar)
					//publish_tf_(d,SONAR_TF);
          range_pubs[d].publish(ranges[d]);
        }

    }

  key = waitKey(1);

  g_lock.leave();
  g_event.set_event();

  return 0;
}

#define RETURN_IF_ERR(err_code) { if( err_code ){ release_transfer(); \
std::cout<<"Error: "<<(e_sdk_err_code)err_code<<" at "<<__LINE__<<","<<__FILE__<<std::endl; return -1;}}

void print_help(){
    printf("Execute like 'rosrun guidance allCameras -cam -son -store /media/ubuntu/USB' \nThis program publish all the cameras in topics (-cam) and sonar info in separate topics (-son) \n press 'q' to quit.\n");
    return;
}

int main(int argc, char** argv)
{
  //printf("the argc %d",argc);
  if(argc>=2){
     for(int i=1;i<argc;i++){
        if(!strcmp(argv[i], "-h")){
            print_help();
            i=999;
            return 0;
        }else if(!strcmp(argv[i], "-cam")){
            printf("Enabling Camera topics.\n");
            enable_cams=true;        
        }else if(!strcmp(argv[i], "-son")){
             printf("Enabling Sonar topics.\n");
             enable_sonars=true;
        }else if(!strcmp(argv[i], "-store")){
            store_images=true;
            if(argc!=i+1){
                if (argv[i+1][0]=='/'){
                    printf("Saving in directory %s",argv[++i]); //to bypass the directory parameter
                    path = argv[i];
                    storage_check();
               }else{
                    printf("Invalid storing path!(not starting with '/\')\n");
                    return 0;
                } 
           }
        }else{
            printf("Unknown parameter/s!\n");
            return 0;
        }
     }
  }

  if(!enable_sonars&&!enable_cams){
    printf("Please enable sonars' or cameras' topics. Add argument -son or -cam.\n");
		return 0;
  }

	leftCamera_pose.position=tf::Vector3(0.01,camerafromcenter,0); leftCamera_pose.rotation=tf::Vector3(0,0,0);
	rightCamera_pose.position=tf::Vector3(0.01,-camerafromcenter,0); rightCamera_pose.rotation=tf::Vector3(0,0,0);
	sonar_pose.position=tf::Vector3(0.01,0,0); sonar_pose.rotation=tf::Vector3(0,0,0);

  /* initialize ros */
  ros::init(argc, argv, "GuidanceCamerasAndSonarOnly");
  ros::NodeHandle my_node;
	
  image_pubs[0] = my_node.advertise<sensor_msgs::Image>("/guidance/down/left_camera/image_raw",1);
  image_pubs[1] = my_node.advertise<sensor_msgs::Image>("/guidance/down/right_camera/image_raw",1);
  image_pubs[2] = my_node.advertise<sensor_msgs::Image>("/guidance/front/left_camera/image_raw",1);
  image_pubs[3] = my_node.advertise<sensor_msgs::Image>("/guidance/front/right_camera/image_raw",1);
  image_pubs[4] = my_node.advertise<sensor_msgs::Image>("/guidance/right/left_camera/image_raw",1);
  image_pubs[5] = my_node.advertise<sensor_msgs::Image>("/guidance/right/right_camera/image_raw",1);
  image_pubs[6] = my_node.advertise<sensor_msgs::Image>("/guidance/rear/left_camera/image_raw",1);
  image_pubs[7] = my_node.advertise<sensor_msgs::Image>("/guidance/rear/right_camera/image_raw",1);
  image_pubs[8] = my_node.advertise<sensor_msgs::Image>("/guidance/left/left_camera/image_raw",1);
  image_pubs[9] = my_node.advertise<sensor_msgs::Image>("/guidance/left/right_camera/image_raw",1);
 
  range_pubs[0] = my_node.advertise<sensor_msgs::Range>("/guidance/down/ultrasonic",1);
  range_pubs[1] = my_node.advertise<sensor_msgs::Range>("/guidance/front/ultrasonic",1);
  range_pubs[2] = my_node.advertise<sensor_msgs::Range>("/guidance/right/ultrasonic",1);
  range_pubs[3] = my_node.advertise<sensor_msgs::Range>("/guidance/rear/ultrasonic",1);
  range_pubs[4] = my_node.advertise<sensor_msgs::Range>("/guidance/left/ultrasonic",1);

	//frame names
  images[0].header.frame_id = "guidanceDown_leftcamera_frame";
  images[1].header.frame_id = "guidanceDown_rightcamera_frame";
  images[2].header.frame_id = "guidanceFront_leftcamera_frame";
  images[3].header.frame_id = "guidanceFront_rightcamera_frame";
  images[4].header.frame_id = "guidanceRight_leftcamera_frame";
  images[5].header.frame_id = "guidanceRight_rightcamera_frame";
  images[6].header.frame_id = "guidanceRear_leftcamera_frame";
  images[7].header.frame_id = "guidanceRear_rightcamera_frame";
  images[8].header.frame_id = "guidanceLeft_leftcamera_frame";
  images[9].header.frame_id = "guidanceLeft_rightcamera_frame";

  ranges[0].header.frame_id = "ultrasonicDown_link";
  ranges[1].header.frame_id = "ultrasonicFront_link";
  ranges[2].header.frame_id = "ultrasonicRight_link";
  ranges[3].header.frame_id = "ultrasonicRear_link";
  ranges[4].header.frame_id = "ultrasonicLeft_link";

//REMOVE THEEESEREMOVE THEEESEREMOVE THEEESEREMOVE THEEESEREMOVE THEEESEREMOVE THEEESEREMOVE THEEESEREMOVE THEEESEREMOVE THEEESE
  /*publish_tf_(1,SONAR_TF);
  publish_tf_(1,CAMERA_TF);
  //postTf_sch_();
  printf("the program ended\n");
  return 0;
  */

  /* initialize guidance */
  reset_config();
  int err_code = init_transfer();
  RETURN_IF_ERR(err_code);

  int online_status[5];
  err_code = get_online_status(online_status);
  RETURN_IF_ERR(err_code);
  std::cout<<"Sensor online status: ";
  for (int i=0; i<5; i++)
    std::cout<<online_status[i]<<" ";
  std::cout<<std::endl;

  // get cali param
  stereo_cali cali[5];
  err_code = get_stereo_cali(cali);
  RETURN_IF_ERR(err_code);
  std::cout<<"cu\tcv\tfocal\tbaseline\n";
  
  for (int i=0; i<5; i++)
  {
    std::cout<<cali[i].cu<<"\t"<<cali[i].cv<<"\t"<<cali[i].focal<<"\t"<<cali[i].baseline<<std::endl;
  }

  /* select data */
  err_code = select_greyscale_image(e_vbus1, true);
  RETURN_IF_ERR(err_code);
  err_code = select_greyscale_image(e_vbus1, false);
  RETURN_IF_ERR(err_code);
  err_code = select_greyscale_image(e_vbus2, true);
  RETURN_IF_ERR(err_code);
  err_code = select_greyscale_image(e_vbus2, false);
  RETURN_IF_ERR(err_code);
  err_code = select_greyscale_image(e_vbus3, true);
  RETURN_IF_ERR(err_code);
  err_code = select_greyscale_image(e_vbus3, false);
  RETURN_IF_ERR(err_code);
  err_code = select_greyscale_image(e_vbus4, true);
  RETURN_IF_ERR(err_code);
  err_code = select_greyscale_image(e_vbus4, false);
  RETURN_IF_ERR(err_code);
  err_code = select_greyscale_image(e_vbus5, true);
  RETURN_IF_ERR(err_code);
  err_code = select_greyscale_image(e_vbus5, false);
  RETURN_IF_ERR(err_code);
  
  select_ultrasonic();
  RETURN_IF_ERR(err_code);
  
  /* start data transfer */
  err_code = set_sdk_event_handler(my_callback);
  RETURN_IF_ERR(err_code);
  err_code = start_transfer();
  RETURN_IF_ERR(err_code);

  std::cout << "start_transfer" << std::endl;	
  ros::Timer timer = my_node.createTimer(ros::Duration(0.1), postTf_sch_); //Posting tf info in fixed time - 10Hz

	while (ros::ok())
  {
    g_event.wait_event();

    ros::spinOnce();
    if (key > 0){
     if (key == 'q'){
        err_code = stop_transfer();
		RETURN_IF_ERR(err_code);
		reset_config();
      break;
    }
  }
}

	/* release data transfer */
err_code = stop_transfer();
RETURN_IF_ERR(err_code);
	//make sure the ack packet from GUIDANCE is received
sleep(1);
std::cout << "release_transfer" << std::endl;
err_code = release_transfer();
RETURN_IF_ERR(err_code);

return 0;
}

/* vim: set et fenc=utf-8 ff=unix sts=0 sw=4 ts=4 : */
