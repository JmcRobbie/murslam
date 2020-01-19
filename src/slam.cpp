#include "ros/ros.h"

#include <fssim_common/Track.h>
#include <sstream>
#include "std_msgs/String.h"

#define rate 1000

int main(int argc, char **argv){
    ros::init(argc, argv, "slam");
    ros::NodeHandle n;

    ros::Publisher chatter_pub = n.advertise<fssim_common::Track>("slam", rate);

    ros::Rate loop_rate(10);

    int count = 0;

    while(ros::ok()){
      std_msgs::String msg;
      std::stringstream ss;
      ss<<"Hello world"<<count;
      msg.data = ss.str();
      ROS_INFO("%s", msg.data.c_str());
      chatter_pub.publish(msg);
      ros::spinOnce();
      loop_rate.sleep();
      ++count;
    }
}
