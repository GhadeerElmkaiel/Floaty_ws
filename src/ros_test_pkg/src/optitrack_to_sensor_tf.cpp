// ROS
#include "ros/ros.h"
#include "geometry_msgs/TransformStamped.h"
#include "geometry_msgs/Transform.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Point.h"
#include "geometry_msgs/Quaternion.h"
#include <tf/transform_broadcaster.h>

//
 
#include <fcntl.h>
#include <stdio.h>
#include <unistd.h>
#include <math.h>
#include <cmath>
 
#ifdef _WIN32
#define O_NOCTTY 0
#else
#include <termios.h>
#endif

#include<unistd.h>
#include <fstream>

void mocapMotorCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
  static tf::TransformBroadcaster br;
  tf::Transform transform;
  transform.setOrigin( tf::Vector3(msg->pose.position.x, msg->pose.position.y, msg->pose.position.z) );
  tf::Quaternion q(msg->pose.orientation.x,  msg->pose.orientation.y,  msg->pose.orientation.z,  msg->pose.orientation.w);
  transform.setRotation(q);
  tf::StampedTransform s_tf(transform, ros::Time::now(), "map", "myframe");
  br.sendTransform(s_tf);
  // br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "map", "myframe"));
  ROS_INFO("send_transform");
  
}

 
int main(int argc, char **argv)
{

  ros::init(argc, argv, "tf_map_ati");
  ros::NodeHandle n;

  ros::Time::init();
  
  // ros::Subscriber sub_motor = n.subscribe("motor", 1000, motorsCallback);
  ros::Subscriber sub_mocapmotor = n.subscribe("/vrpn_client_node/ATISensor/pose", 1000, mocapMotorCallback);

  

  ros::spin();

  return 0;
}