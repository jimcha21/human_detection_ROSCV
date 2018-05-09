#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "ros/ros.h"
#include "sensor_msgs/Image.h"
#include "std_srvs/Empty.h"
#include "sensor_msgs/CameraInfo.h"
#include "sensor_msgs/SetCameraInfo.h"
#include "camera_info_manager/camera_info_manager.h"

sensor_msgs::CameraInfo c_info;

int main(int argc, char **argv){
   ros::init(argc, argv, "raspicam_raw_node");
   ros::NodeHandle n;
   camera_info_manager::CameraInfoManager c_info_man (n, "/guidance/rear/right_camera", "package://guidance/calibration_files/camera_params_right.yaml");

		if(!c_info_man.loadCameraInfo ("package://guidance/calibration_files/camera_params_right.yaml")){
				ROS_INFO("Calibration file missing. Camera not calibrated");
				 }
				 else
				 {
					c_info = c_info_man.getCameraInfo ();
					 
				ROS_INFO("Camera successfully calibrated");
				 }
				 
				 
 //  ros::Publisher image_pub = n.advertise<sensor_msgs::Image>("/guidance/left/right_camera", 1);
   ros::Publisher camera_info_pub = n.advertise<sensor_msgs::CameraInfo>("/guidance/rear/right_camera/camera_info", 1);
   /*ros::ServiceServer start_cam = n.advertiseService("camera/start_capture", serv_start_cap);
   ros::ServiceServer stop_cam = n.advertiseService("camera/stop_capture", serv_stop_cap);
   */
				 

					 
				 ros::spin();
				 
   return 0;
}