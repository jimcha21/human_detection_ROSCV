#include <ros/ros.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/tf.h>

double counter = 0;

int main (int argc, char** argv) 
{
    ros::init(argc, argv, "initialpose_pub");
    ros::NodeHandle nh_;
    ros::Publisher initPosePub_ = nh_.advertise<geometry_msgs::PoseWithCovarianceStamped>("/initialpose", 2, true);
        ros::Publisher goal_ = nh_.advertise<geometry_msgs::PoseStamped>("/goal", 2, true);

ros::Rate rate(1.0);
while(nh_.ok()) 
{
    counter++;
    //get time
    ros::Time scanTime_ = ros::Time::now(); 
    //  scanTime_.setNow(1,1);
    //create msg
    geometry_msgs::PoseWithCovarianceStamped initPose_;
    //create the time & frame
    initPose_.header.stamp = scanTime_;
    initPose_.header.frame_id = "map";
    //position
    initPose_.pose.pose.position.x = counter;
    initPose_.pose.pose.position.y = 0.f;
    initPose_.pose.pose.position.z = 0.f;
    //angle
    initPose_.pose.pose.orientation.x = 0.f;
    initPose_.pose.pose.orientation.y = 0.f;
    initPose_.pose.pose.orientation.z = 0.f;
    initPose_.pose.pose.orientation.w = 1.f;
    //initPose_.pose.pose.orientation.z = 0.f;
    //publish msg
    initPosePub_.publish(initPose_);
    if(counter==10.0f)
        counter=0;

    geometry_msgs::PoseStamped goalPose;

    goalPose.header.stamp = scanTime_;
    goalPose.header.frame_id = "map";
    goalPose.pose.position.x = 1.2f;
    goalPose.pose.position.y = 1.f;
    goalPose.pose.position.z = 0.f;
    //angle
    goalPose.pose.orientation.x = 0.f;
    goalPose.pose.orientation.y = 0.f;
    goalPose.pose.orientation.z = 0.f;
    goalPose.pose.orientation.w = 1.f;

    //goal_.publish(goalPose);
    //sleep
    rate.sleep();
}
return 0;
}