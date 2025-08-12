
#include "ros/ros.h"
#include "geometry_msgs/PoseStamped.h"

#include<unistd.h>
#include <fstream>

#include <chrono>
#include <iostream>
#include <sys/time.h>
#include <ctime>

using std::cout; using std::endl;
using std::chrono::duration_cast;
using std::chrono::milliseconds;
using std::chrono::seconds;
using std::chrono::system_clock;

auto start_millisec = duration_cast<milliseconds>(system_clock::now().time_since_epoch()).count();

void mocapCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    // Another way to check the time of recieving the data 
    // using ROS_INFO() as it has time included
    // ROS_INFO("Got new point");

    auto millisec_since_epoch = duration_cast<milliseconds>(system_clock::now().time_since_epoch()).count();

    // Record the time of recieving the data from the Optitrack
    std::fstream fout;
    fout.open("/home/floaty/data.txt", std::ios::out | std::ios::app);
    fout<< "milliseconds: "<<  millisec_since_epoch - start_millisec <<"\n";

    fout.close();
}

int main(int argc, char **argv)
{
    start_millisec = duration_cast<milliseconds>(system_clock::now().time_since_epoch()).count();

    ros::init(argc, argv, "data_time_recorder_node");

    ros::NodeHandle nh;

    ros::Time::init();

    ros::Subscriber sub_mocap = nh.subscribe("/vrpn_client_node/ATISensor/pose", 1000, mocapCallback);

    ros::spin();

    return 0;
}