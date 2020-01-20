#include "ros/ros.h"
#include <fssim_common/State.h>
#include <fssim_common/Track.h>
#include <sstream>
#include "std_msgs/String.h"
#include "sensor_msgs/PointCloud2.h"

#define rate 1000
#define looprate 10 

void lidarCallback(const sensor_msgs::PointCloud2::ConstPtr& msg);
void cameraCallback(const sensor_msgs::PointCloud2::ConstPtr& msg);

int main(int argc, char **argv){
    ros::init(argc, argv, "slam");
    /* Node handle for accessing ros core */  
    ros::NodeHandle n;

    /* Publishers*/

    ros::Publisher slam_pub = n.advertise<fssim_common::Track>("slam", rate);
    ros::Publisher talker = n.advertise<std_msgs::String>("talker", rate);

    /*Listeners*/

    ros::Subscriber lidar_subs = n.subscribe("/lidar/cones",rate,lidarCallback);
    ros::Subscriber camera_sub = n.subscribe("/camera/cones",rate,cameraCallback);

    ros::Rate loop_rate(looprate);
 
    int count = 0;


    while(ros::ok()){
      std_msgs::String msg;
      std::stringstream ss;
      ss<<"Hello world"<<count;
      msg.data = ss.str();
      ROS_INFO("%s", msg.data.c_str());
      talker.publish(msg);
      ros::spinOnce();
      loop_rate.sleep();
      ++count;
    }
}

void lidarCallback(const sensor_msgs::PointCloud2::ConstPtr& msg){
  /* Placeholder function, will be replaced later.*/
  }

void CameraCallback(const sensor_msgs::PointCloud2::ConstPtr& msg){
/* Placeholder function, will be replaced later.*/
}