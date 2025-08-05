/*********************************************************************
 * This code is a part of Floaty project
 * 
 *********************************************************************/

/**
 * Simple stand-alone ROS node that record data from different topics
 * and savethem in a file
 */

#include "ros/ros.h"
#include "geometry_msgs/TransformStamped.h"
#include "geometry_msgs/Transform.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/WrenchStamped.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Point.h"
#include "geometry_msgs/Quaternion.h"
// #include "floaty_msgs"
#include<floaty_msgs/record_data_srv.h>


#include<unistd.h>
#include <fstream>

std::string data_topic, mocap_topic, data_file_name;
int num_data_to_record;
double record_duration;

class DataCollector
{
    private:
        float x_pos, y_pos, z_pos, x_force, y_force, z_force, x_torque, y_torque, z_torque;
        int data_to_record;
        int num_of_data_points;
        std::string file_name;
        bool recording_done;
        float x_pos_arr[2000];
        float y_pos_arr[2000];
        float z_pos_arr[2000];
        float x_force_arr[2000];
        float y_force_arr[2000];
        float z_force_arr[2000];
        float x_torque_arr[2000];
        float y_torque_arr[2000];
        float z_torque_arr[2000];
    public:
    // Constructor function
    DataCollector() : x_pos(0), y_pos(0), z_pos(0), x_force(0), y_force(0), z_force(0), x_torque(0), y_torque(0), z_torque(0), data_to_record(0), num_of_data_points(0), file_name(data_file_name), recording_done(false) { }

        void forceCallback(const geometry_msgs::WrenchStamped::ConstPtr& msg)
        {
            x_force = msg->wrench.force.x;
            y_force = msg->wrench.force.y;
            z_force = msg->wrench.force.z;
            x_torque = msg->wrench.torque.x;
            y_torque = msg->wrench.torque.y;
            z_torque = msg->wrench.torque.z;
        }

        void mocapCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
        {
            x_pos = msg->pose.position.x;
            y_pos = msg->pose.position.y;
            z_pos = msg->pose.position.z;
        }

        bool activateTimerServiceCallback(floaty_msgs::record_data_srv::Request& request, floaty_msgs::record_data_srv::Response& response)
        {
            data_to_record = request.data_to_record;
            num_of_data_points = request.data_to_record;
            if(data_to_record>0)
            {
                ROS_INFO("Recording Started");
                recording_done=false;
                file_name = request.file_name;
            }
            while(!recording_done)
            {
                sleep(0.01);
            }
            return true;
        }

        void timerCallback(const ros::TimerEvent&)
        {

            if(data_to_record==0)
                return;
            
            std::fstream fout;
            // fout.open("motors.csv", std::ios::out | std::ios::app);
            fout.open(file_name, std::ios::out | std::ios::app);
            fout<< x_pos << ", "<< y_pos << ", "<< z_force <<"\n";

            fout.close();

            data_to_record = data_to_record-1;
            if(data_to_record==0)
            {
                recording_done = true;
                ROS_INFO("Recording Ended");
            }
        }

};


int main(int argc, char **argv)
{

    ros::init(argc, argv, "data_recorder_node");
    ros::NodeHandle nh;

    nh.param<std::string>(nh.getNamespace() + "/data_topic",data_topic, "/ft_sensor/netft_data");
    nh.param<std::string>(nh.getNamespace() + "/data_file_name",data_file_name, "/recorded_data.csv");
    nh.param<std::string>(nh.getNamespace() + "/mocap_topic",mocap_topic, "/vrpn_client_node/ATISensor/pose");

    nh.getParam(nh.getNamespace() + "/data_to_record",num_data_to_record);
    nh.getParam(nh.getNamespace() + "/record_duration",record_duration);


    // ROS_INFO(data_file_name);

    ros::Time::init();

    DataCollector data_collector_object;
    ros::AsyncSpinner spinner(4);
    spinner.start();
    
    ros::Subscriber sub_force = nh.subscribe(data_topic, 1000, &DataCollector::forceCallback, &data_collector_object);
    ros::Subscriber sub_mocap = nh.subscribe(mocap_topic, 1000, &DataCollector::mocapCallback, &data_collector_object);

    ros::Timer collect_data_timer = nh.createTimer(ros::Duration(record_duration), &DataCollector::timerCallback,  &data_collector_object);

    ros::ServiceServer srv_record_data = nh.advertiseService("collect_data_service", &DataCollector::activateTimerServiceCallback, &data_collector_object);

    ros::waitForShutdown();

    return 0;
}