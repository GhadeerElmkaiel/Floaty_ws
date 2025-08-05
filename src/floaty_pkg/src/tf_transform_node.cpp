/*********************************************************************
 * This code is a part of Floaty project
 * 
 *********************************************************************/

/**
 * Simple stand-alone ROS node that takes data from PoseStamped topic
 * and use it to broadcast a tf transform between to frames 
 */

#include "ros/ros.h"
#include "geometry_msgs/TransformStamped.h"
#include "geometry_msgs/Transform.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Point.h"
#include "geometry_msgs/Quaternion.h"
#include <tf/transform_broadcaster.h>


std::string base_frame, trans_frame;

void mocapCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{


    static tf::TransformBroadcaster br;
    tf::Transform transform;
    transform.setOrigin( tf::Vector3(msg->pose.position.x, msg->pose.position.y, msg->pose.position.z) );
    tf::Quaternion q(msg->pose.orientation.x,  msg->pose.orientation.y,  msg->pose.orientation.z,  msg->pose.orientation.w);
    transform.setRotation(q);
    tf::StampedTransform s_tf(transform, ros::Time::now(), base_frame, trans_frame);
    br.sendTransform(s_tf);
  
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "tf_transform_node");

    ros::NodeHandle nh;
    std::string transform_topic;

    // nh.param<std::string>(nh.getNamespace() + "/base_frame",base_frame, "map");
    // nh.param<std::string>(nh.getNamespace() + "/trans_frame",trans_frame, "");
    nh.param<std::string>("/TF/base_frame",base_frame, "/map");
    nh.param<std::string>("/TF/trans_frame",trans_frame, "/sensor");

    // nh.param<std::string>(nh.getNamespace() + "/transform_topic",transform_topic, "/vrpn_client_node/ATISensor/pose");
    nh.param<std::string>("/NatNet/data_topic",transform_topic, "/Optitrack/Floaty");
    transform_topic = transform_topic;

    ros::Time::init();

    ros::Subscriber sub_mocap = nh.subscribe(transform_topic, 1000, mocapCallback);

    ros::spin();

    return 0;
}