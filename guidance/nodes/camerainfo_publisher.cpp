#include "ros/ros.h"
#include "sensor_msgs/CameraInfo.h"

#include <iostream>
#include <sstream>
#include "yaml-cpp/yaml.h"

using namespace std;

sensor_msgs::CameraInfo front_leftcamera_caminf,front_rightcamera_caminf;
sensor_msgs::CameraInfo right_leftcamera_caminf,right_rightcamera_caminf;
sensor_msgs::CameraInfo left_leftcamera_caminf,left_rightcamera_caminf;
sensor_msgs::CameraInfo rear_leftcamera_caminf,rear_rightcamera_caminf;
sensor_msgs::CameraInfo down_leftcamera_caminf,down_rightcamera_caminf;
string yml_directory = "/home/ubuntu/catkin_ws/src/human_detection/guidance/calibration_results";
bool enable_front_camerainfo,enable_rear_camerainfo,enable_down_camerainfo,enable_left_camerainfo,enable_right_camerainfo;

static const char CAMERAMATRIX_LEFT[]           = "M1";
static const char DISTORTIONPARAMETERS_LEFT[]   = "D1";
static const char CAMERAMATRIX_RIGHT[]          = "M2";
static const char DISTORTIONPARAMETERS_RIGHT[]  = "D2";

static const char RECTIFICATIONMATRIX_LEFT[]    = "R1";
static const char RECTIFICATIONMATRIX_RIGHT[]   = "R2";
static const char PROJECTIONMATRIX_LEFT[]       = "P1";
static const char PROJECTIONMATRIX_RIGHT[]      = "P2";

void storeMatsfromYML(const YAML::Node& node, double* pointerToCamInfoMat){

    double* buffer = (double*) malloc (3*sizeof(double));

    int rows, cols;
    rows = node["rows"].as<int>();
    cols = node["cols"].as<int>();
    const YAML::Node& data = node["data"];
    for (int i = 0; i < rows*cols; ++i)
    {
        pointerToCamInfoMat[i] = data[i].as<double>();
    }
} 

void pushMatsfromYML(const YAML::Node& node, std::vector<double>& pointerToCamInfoMat){

    double* buffer = (double*) malloc (3*sizeof(double));

    int rows, cols;
    rows = node["rows"].as<int>();
    cols = node["cols"].as<int>();
    const YAML::Node& data = node["data"];
    for (int i = 0; i < rows*cols; ++i)
    {
        pointerToCamInfoMat.push_back(data[i].as<double>()) ;
    }
} 

int main(int argc, char **argv)
{
  ros::init(argc, argv, "camerainfo_publisher");

  if( argc <= 1)
  {
    cout <<"Please insert at least one param (left,right,rear,front,down)" << endl;
    return -1;
  }

  enable_front_camerainfo = enable_rear_camerainfo = enable_down_camerainfo = enable_left_camerainfo = enable_right_camerainfo = false;

  for (int i = 1; i < argc; ++i)
  {
    if(!strcmp(argv[i],"front"))
      enable_front_camerainfo=true;
    if(!strcmp(argv[i],"rear"))
      enable_rear_camerainfo=true;
    if(!strcmp(argv[i],"left"))
      enable_left_camerainfo=true;
    if(!strcmp(argv[i],"right"))
      enable_right_camerainfo=true;
    if(!strcmp(argv[i],"down"))
      enable_down_camerainfo=true;
  }

  ros::NodeHandle n;
  ros::Publisher frontcmrinfo_pub_l = n.advertise<sensor_msgs::CameraInfo>("/guidance/front/left_camera/camera_info", 1000);
  ros::Publisher frontcmrinfo_pub_r = n.advertise<sensor_msgs::CameraInfo>("/guidance/front/right_camera/camera_info", 1000);
  ros::Publisher rightcmrinfo_pub_l = n.advertise<sensor_msgs::CameraInfo>("/guidance/right/left_camera/camera_info", 1000);
  ros::Publisher rightcmrinfo_pub_r = n.advertise<sensor_msgs::CameraInfo>("/guidance/right/right_camera/camera_info", 1000);
  ros::Publisher leftcmrinfo_pub_l = n.advertise<sensor_msgs::CameraInfo>("/guidance/left/left_camera/camera_info", 1000);
  ros::Publisher leftcmrinfo_pub_r = n.advertise<sensor_msgs::CameraInfo>("/guidance/left/right_camera/camera_info", 1000);
  ros::Publisher downcmrinfo_pub_l = n.advertise<sensor_msgs::CameraInfo>("/guidance/down/left_camera/camera_info", 1000);
  ros::Publisher downcmrinfo_pub_r = n.advertise<sensor_msgs::CameraInfo>("/guidance/down/right_camera/camera_info", 1000);
  ros::Publisher rearcmrinfo_pub_l = n.advertise<sensor_msgs::CameraInfo>("/guidance/rear/left_camera/camera_info", 1000);
  ros::Publisher rearcmrinfo_pub_r = n.advertise<sensor_msgs::CameraInfo>("/guidance/rear/right_camera/camera_info", 1000);

  YAML::Node intrinsics_yml_front,extrinsics_yml_front,intrinsics_yml_right,extrinsics_yml_right,
    intrinsics_yml_left,extrinsics_yml_left,intrinsics_yml_rear,extrinsics_yml_rear,intrinsics_yml_down,extrinsics_yml_down;

  //front cameras
  if(enable_front_camerainfo){
    try{
      intrinsics_yml_front = YAML::LoadFile(yml_directory+"/intrinsics_front.yaml");
      extrinsics_yml_front = YAML::LoadFile(yml_directory+"/extrinsics_front.yaml");
    }catch(const std::runtime_error& e){
      cout<<"Error opening the files ["<<e.what()<<"]"<<endl;
    }

    front_leftcamera_caminf.header.frame_id = front_rightcamera_caminf.header.frame_id = "guidanceFront_link";
    front_leftcamera_caminf.header.stamp = front_rightcamera_caminf.header.stamp = ros::Time::now();
    front_leftcamera_caminf.height = front_rightcamera_caminf.height = 240;
    front_leftcamera_caminf.width = front_rightcamera_caminf.width =320;
    front_leftcamera_caminf.distortion_model = front_rightcamera_caminf.distortion_model = "plumb_bob";   

    storeMatsfromYML(intrinsics_yml_front[CAMERAMATRIX_LEFT],&front_leftcamera_caminf.K[0]);
    pushMatsfromYML(intrinsics_yml_front[DISTORTIONPARAMETERS_LEFT],front_leftcamera_caminf.D);
    storeMatsfromYML(extrinsics_yml_front[RECTIFICATIONMATRIX_LEFT],&front_leftcamera_caminf.R[0]);
    storeMatsfromYML(extrinsics_yml_front[PROJECTIONMATRIX_LEFT],&front_leftcamera_caminf.P[0]);

    storeMatsfromYML(intrinsics_yml_front[CAMERAMATRIX_RIGHT],&front_rightcamera_caminf.K[0]);
    pushMatsfromYML(intrinsics_yml_front[DISTORTIONPARAMETERS_RIGHT],front_rightcamera_caminf.D);
    storeMatsfromYML(extrinsics_yml_front[RECTIFICATIONMATRIX_RIGHT],&front_rightcamera_caminf.R[0]);
    storeMatsfromYML(extrinsics_yml_front[PROJECTIONMATRIX_RIGHT],&front_rightcamera_caminf.P[0]);
  }

  //right cameras
  if(enable_right_camerainfo){
    try{
      intrinsics_yml_right = YAML::LoadFile(yml_directory+"/intrinsics_right.yaml");
      extrinsics_yml_right = YAML::LoadFile(yml_directory+"/extrinsics_right.yaml");
    }catch(const std::runtime_error& e){
      cout<<"Error opening the files ["<<e.what()<<"]"<<endl;
    }

    right_leftcamera_caminf.header.frame_id = right_rightcamera_caminf.header.frame_id = "guidanceRight_link";
    right_leftcamera_caminf.header.stamp = right_rightcamera_caminf.header.stamp = ros::Time::now();
    right_leftcamera_caminf.height = right_rightcamera_caminf.height = 240;
    right_leftcamera_caminf.width = right_rightcamera_caminf.width =320;
    right_leftcamera_caminf.distortion_model = right_rightcamera_caminf.distortion_model = "plumb_bob";   

    storeMatsfromYML(intrinsics_yml_right[CAMERAMATRIX_LEFT],&right_leftcamera_caminf.K[0]);
    pushMatsfromYML(intrinsics_yml_right[DISTORTIONPARAMETERS_LEFT],right_leftcamera_caminf.D);
    storeMatsfromYML(extrinsics_yml_right[RECTIFICATIONMATRIX_LEFT],&right_leftcamera_caminf.R[0]);
    storeMatsfromYML(extrinsics_yml_right[PROJECTIONMATRIX_LEFT],&right_leftcamera_caminf.P[0]);

    storeMatsfromYML(intrinsics_yml_right[CAMERAMATRIX_RIGHT],&right_rightcamera_caminf.K[0]);
    pushMatsfromYML(intrinsics_yml_right[DISTORTIONPARAMETERS_RIGHT],right_rightcamera_caminf.D);
    storeMatsfromYML(extrinsics_yml_right[RECTIFICATIONMATRIX_RIGHT],&right_rightcamera_caminf.R[0]);
    storeMatsfromYML(extrinsics_yml_right[PROJECTIONMATRIX_RIGHT],&right_rightcamera_caminf.P[0]);
  }


  //left cameras
  if(enable_left_camerainfo){
    try{
      intrinsics_yml_left = YAML::LoadFile(yml_directory+"/intrinsics_left.yaml");
      extrinsics_yml_left = YAML::LoadFile(yml_directory+"/extrinsics_left.yaml");
    }catch(const std::runtime_error& e){
      cout<<"Error opening the files ["<<e.what()<<"]"<<endl;
    }

    left_leftcamera_caminf.header.frame_id = left_rightcamera_caminf.header.frame_id = "guidanceLeft_link";
    left_leftcamera_caminf.header.stamp = left_rightcamera_caminf.header.stamp = ros::Time::now();
    left_leftcamera_caminf.height = left_rightcamera_caminf.height = 240;
    left_leftcamera_caminf.width = left_rightcamera_caminf.width =320;
    left_leftcamera_caminf.distortion_model = left_rightcamera_caminf.distortion_model = "plumb_bob";   

    storeMatsfromYML(intrinsics_yml_left[CAMERAMATRIX_LEFT],&left_leftcamera_caminf.K[0]);
    pushMatsfromYML(intrinsics_yml_left[DISTORTIONPARAMETERS_LEFT],left_leftcamera_caminf.D);
    storeMatsfromYML(extrinsics_yml_left[RECTIFICATIONMATRIX_LEFT],&left_leftcamera_caminf.R[0]);
    storeMatsfromYML(extrinsics_yml_left[PROJECTIONMATRIX_LEFT],&left_leftcamera_caminf.P[0]);

    storeMatsfromYML(intrinsics_yml_left[CAMERAMATRIX_RIGHT],&left_rightcamera_caminf.K[0]);
    pushMatsfromYML(intrinsics_yml_left[DISTORTIONPARAMETERS_RIGHT],left_rightcamera_caminf.D);
    storeMatsfromYML(extrinsics_yml_left[RECTIFICATIONMATRIX_RIGHT],&left_rightcamera_caminf.R[0]);
    storeMatsfromYML(extrinsics_yml_left[PROJECTIONMATRIX_RIGHT],&left_rightcamera_caminf.P[0]);
  }


  //down cameras
  if(enable_down_camerainfo){
      try{
        intrinsics_yml_down = YAML::LoadFile(yml_directory+"/intrinsics_down.yaml");
        extrinsics_yml_down = YAML::LoadFile(yml_directory+"/extrinsics_down.yaml");
      }catch(const std::runtime_error& e){
        cout<<"Error opening the files ["<<e.what()<<"]"<<endl;
      }

    down_leftcamera_caminf.header.frame_id = down_rightcamera_caminf.header.frame_id = "guidanceDown_link";
    down_leftcamera_caminf.header.stamp = down_rightcamera_caminf.header.stamp = ros::Time::now();
    down_leftcamera_caminf.height = down_rightcamera_caminf.height = 240;
    down_leftcamera_caminf.width = down_rightcamera_caminf.width =320;
    down_leftcamera_caminf.distortion_model = down_rightcamera_caminf.distortion_model = "plumb_bob";   

    storeMatsfromYML(intrinsics_yml_down[CAMERAMATRIX_LEFT],&down_leftcamera_caminf.K[0]);
    pushMatsfromYML(intrinsics_yml_down[DISTORTIONPARAMETERS_LEFT],down_leftcamera_caminf.D);
    storeMatsfromYML(extrinsics_yml_down[RECTIFICATIONMATRIX_LEFT],&down_leftcamera_caminf.R[0]);
    storeMatsfromYML(extrinsics_yml_down[PROJECTIONMATRIX_LEFT],&down_leftcamera_caminf.P[0]);

    storeMatsfromYML(intrinsics_yml_down[CAMERAMATRIX_RIGHT],&down_rightcamera_caminf.K[0]);
    pushMatsfromYML(intrinsics_yml_down[DISTORTIONPARAMETERS_RIGHT],down_rightcamera_caminf.D);
    storeMatsfromYML(extrinsics_yml_down[RECTIFICATIONMATRIX_RIGHT],&down_rightcamera_caminf.R[0]);
    storeMatsfromYML(extrinsics_yml_down[PROJECTIONMATRIX_RIGHT],&down_rightcamera_caminf.P[0]);
  }
  

  //rear cameras
  if(enable_rear_camerainfo){
    try{
      intrinsics_yml_rear = YAML::LoadFile(yml_directory+"/intrinsics_rear.yaml");
      extrinsics_yml_rear = YAML::LoadFile(yml_directory+"/extrinsics_rear.yaml");
    }catch(const std::runtime_error& e){
      cout<<"Error opening the files ["<<e.what()<<"]"<<endl;
    }
    rear_leftcamera_caminf.header.frame_id = rear_rightcamera_caminf.header.frame_id = "guidanceRear_link";
    rear_leftcamera_caminf.header.stamp = rear_rightcamera_caminf.header.stamp = ros::Time::now();
    rear_leftcamera_caminf.height = rear_rightcamera_caminf.height = 240;
    rear_leftcamera_caminf.width = rear_rightcamera_caminf.width =320;
    rear_leftcamera_caminf.distortion_model = rear_rightcamera_caminf.distortion_model = "plumb_bob";   

    storeMatsfromYML(intrinsics_yml_rear[CAMERAMATRIX_LEFT],&rear_leftcamera_caminf.K[0]);
    pushMatsfromYML(intrinsics_yml_rear[DISTORTIONPARAMETERS_LEFT],rear_leftcamera_caminf.D);
    storeMatsfromYML(extrinsics_yml_rear[RECTIFICATIONMATRIX_LEFT],&rear_leftcamera_caminf.R[0]);
    storeMatsfromYML(extrinsics_yml_rear[PROJECTIONMATRIX_LEFT],&rear_leftcamera_caminf.P[0]);

    storeMatsfromYML(intrinsics_yml_rear[CAMERAMATRIX_RIGHT],&rear_rightcamera_caminf.K[0]);
    pushMatsfromYML(intrinsics_yml_rear[DISTORTIONPARAMETERS_RIGHT],rear_rightcamera_caminf.D);
    storeMatsfromYML(extrinsics_yml_rear[RECTIFICATIONMATRIX_RIGHT],&rear_rightcamera_caminf.R[0]);
    storeMatsfromYML(extrinsics_yml_rear[PROJECTIONMATRIX_RIGHT],&rear_rightcamera_caminf.P[0]);
  }

  ros::Rate loop_rate(35); //no need to rush

  while (ros::ok())
  {

    //cout<<"sending camera info for front"<<endl;
    if(enable_front_camerainfo){
      frontcmrinfo_pub_l.publish(front_leftcamera_caminf);
      frontcmrinfo_pub_r.publish(front_rightcamera_caminf);
    }
    if(enable_right_camerainfo){
      rightcmrinfo_pub_l.publish(right_leftcamera_caminf);
      rightcmrinfo_pub_r.publish(right_rightcamera_caminf);
    }
    if(enable_left_camerainfo){
      leftcmrinfo_pub_l.publish(left_leftcamera_caminf);
      leftcmrinfo_pub_r.publish(left_rightcamera_caminf);
    }
    if(enable_rear_camerainfo){
      rearcmrinfo_pub_l.publish(rear_leftcamera_caminf);
      rearcmrinfo_pub_r.publish(rear_rightcamera_caminf);
    }
    if(enable_down_camerainfo){
      downcmrinfo_pub_l.publish(down_leftcamera_caminf);
      downcmrinfo_pub_r.publish(down_rightcamera_caminf);
    }

    ros::spinOnce();
    loop_rate.sleep();
  }


  return 0;
}
