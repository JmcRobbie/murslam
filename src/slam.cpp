#include "ros/ros.h"
#include <fssim_common/State.h>
#include <fssim_common/Track.h>
#include <sstream>
#include "std_msgs/String.h"
#include "sensor_msgs/PointCloud2.h"

#define rate 1000

void basiccallback(const sensor_msgs::PointCloud2::ConstPtr& msg);

int main(int argc, char **argv){
    ros::init(argc, argv, "slam");

    ros::NodeHandle n;
    /* Publishers*/

    ros::Publisher slam_pub = n.advertise<fssim_common::Track>("slam", rate);
    ros::Publisher talker = n.advertise<std_msgs::String>("talker", rate);

    /*Listeners*/

    ros::Subscriber lidar_subs = n.subscribe("/lidar/cones",rate,basiccallback);
    ros::Subscriber camera_sub = n.subscribe("/camera/cones",rate,basiccallback);

    ros::Rate loop_rate(10);
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

void basiccallback(const sensor_msgs::PointCloud2::ConstPtr& msg){

/* Placeholder function, will be replaced later.*/
}