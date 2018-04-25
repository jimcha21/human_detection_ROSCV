#include <stdio.h>
#include <string.h>
#include <iostream>
#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>

#include <opencv2/opencv.hpp>

#include <geometry_msgs/TransformStamped.h> //IMU
#include <geometry_msgs/Vector3Stamped.h> //velocity
#include <sensor_msgs/LaserScan.h> //obstacle distance & ultrasonic
#include <sensor_msgs/CameraInfo.h> // camera info message. Contains cam params
#include "yaml-cpp/yaml.h" // use to parse YAML calibration file
#include <fstream> // required to parse YAML 

using namespace std;
ros::Publisher mpee; // camera info msg publishers

static const char CAM_YML_NAME[]    = "camera_name";
static const char WIDTH_YML_NAME[]  = "image_width";
static const char HEIGHT_YML_NAME[] = "image_height";
static const char K_YML_NAME[]      = "camera_matrix";
static const char D_YML_NAME[]      = "distortion_coefficients";
static const char R_YML_NAME[]      = "rectification_matrix";
static const char P_YML_NAME[]      = "projection_matrix";
static const char DMODEL_YML_NAME[] = "distortion_model";

struct SimpleMatrix
{
    int rows;
    int cols;
    double* data;
    SimpleMatrix(int rows, int cols, double* data)
   : rows(rows), cols(cols), data(data)
   {}
};

void transfer_SimpleMatrix_from_YML_to_ROSmsg(const YAML::Node& node, SimpleMatrix& m)
{
    int rows, cols;
    rows = node["rows"].as<int>();
    cols = node["cols"].as<int>();
    const YAML::Node& data = node["data"];
    for (int i = 0; i < rows*cols; ++i)
    {
       m.data[i] = data[i].as<double>();
    }
}

void read_params_from_yaml_and_fill_cam_info_msg(std::string& file_name, sensor_msgs::CameraInfo& cam_info)
{
   std::ifstream fin(file_name.c_str());
   YAML::Node doc = YAML::Load(fin);

   cam_info.width = doc[WIDTH_YML_NAME].as<int>();
   cam_info.height = doc[HEIGHT_YML_NAME].as<int>();
     
   SimpleMatrix K_(3, 3, &cam_info.K[0]);
   transfer_SimpleMatrix_from_YML_to_ROSmsg(doc[K_YML_NAME], K_);

   SimpleMatrix R_(3, 3, &cam_info.R[0]);
   transfer_SimpleMatrix_from_YML_to_ROSmsg(doc[R_YML_NAME], R_);

   SimpleMatrix P_(3, 4, &cam_info.P[0]);
   transfer_SimpleMatrix_from_YML_to_ROSmsg(doc[P_YML_NAME], P_);

   cam_info.distortion_model = doc[DMODEL_YML_NAME].as<std::string>();

   const YAML::Node& D_node = doc[D_YML_NAME];
   int D_rows, D_cols;

   D_rows = D_node["rows"].as<int>();
   D_cols = D_node["cols"].as<int>();
   const YAML::Node& D_data = D_node["data"];
   cam_info.D.resize(D_rows*D_cols);

   for (int i = 0; i < D_rows*D_cols; ++i)
   {
      cam_info.D[i] = D_data[i].as<float>();
   }
   cout<<cam_info<<endl;
}


int main(int argc, char const *argv[])
{
	sensor_msgs::CameraInfo cam_info_topic;
	std::string dir = "/home/ubuntu/catkin_ws/src/human_detection/guidance/calibration_files/camera_params_left.yaml";
	read_params_from_yaml_and_fill_cam_info_msg(dir , cam_info_topic);

	return 0;
}
