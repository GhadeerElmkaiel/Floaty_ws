/*********************************************************************
 * This code is a part of Floaty project
 * 
 *********************************************************************/

/**
 * Simple stand-alone ROS node that record data from vrpn topic
 * and save it in a file
 */

#include "ros/ros.h"

#include "geometry_msgs/PoseStamped.h"

#include<floaty_msgs/record_data_srv.h>
// // #include "netft_rdt_driver/netft_rdt_driver.h"
// #include "netft_rdt_driver/netft_rdt_bias.h"

#include<unistd.h>
#include <fstream>

std::string data_topic, mocap_topic, data_file_path;
double record_duration;
bool use_data_directly;


class DataCollector
{
    private:
        float x_pos_arr[40000];
        float y_pos_arr[40000];
        float z_pos_arr[40000];
        float x_rot_arr[40000];
        float y_rot_arr[40000];
        float z_rot_arr[40000];
        float w_rot_arr[40000];
        float x_pos, y_pos, z_pos, x_rot, y_rot, z_rot, w_rot;
        int data_to_record;
        std::string file_name;
        std::string file_path;
        bool recording_done;
        int counter;
        bool receiver_sensor_data;
        bool use_data;
    public:
    // Constructor function
    DataCollector() : x_pos(0), y_pos(0), z_pos(0), x_rot(0), y_rot(0), z_rot(0), w_rot(0), data_to_record(0), file_path(data_file_path), file_name("data.csv"), recording_done(false), counter(0), receiver_sensor_data(false), use_data(use_data_directly) { }

        void forceCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
        {
            receiver_sensor_data = true;
            if(data_to_record==0)
                return;
            if(use_data){               // Recording directly when recieving data from sensor
                // receiver_sensor_data = true;
                x_pos_arr[counter] = msg->pose.position.x;
                y_pos_arr[counter] = msg->pose.position.y;
                z_pos_arr[counter] = msg->pose.position.z;
                x_rot_arr[counter] = msg->pose.orientation.x;
                y_rot_arr[counter] = msg->pose.orientation.y;
                z_rot_arr[counter] = msg->pose.orientation.z;
                w_rot_arr[counter] = msg->pose.orientation.w;

                data_to_record = data_to_record-1;
                counter = counter+1;
                if(data_to_record==0)
                {
                    std::string full_file_path;
                    full_file_path = file_path + file_name;
                    std::fstream fout;
                    fout.open(full_file_path, std::ios::out | std::ios::app);
                    for(int i=0; i<counter; i++){
                        fout<< x_pos_arr[i] << ", "<< y_pos_arr[i] << ", "<< z_pos_arr[i] << ", ";
                        fout<< x_rot_arr[i] << ", "<< y_rot_arr[i] << ", "<< z_rot_arr[i] << ", "<< w_rot_arr[i] <<"\n";
                    }

                    fout.close();

                    recording_done = true;
                    // ROS_INFO("Recording Ended");
                    counter = 0;
                }
            }
            else{
                // receiver_sensor_data = true;
                x_pos = msg->pose.position.x;
                y_pos = msg->pose.position.y;
                z_pos = msg->pose.position.z;
                x_rot = msg->pose.orientation.x;
                y_rot = msg->pose.orientation.y;
                z_rot = msg->pose.orientation.z;
                w_rot = msg->pose.orientation.w;
            }
        }

        bool activateTimerServiceCallback(floaty_msgs::record_data_srv::Request& request, floaty_msgs::record_data_srv::Response& response)
        {
            if(receiver_sensor_data==false){
                ROS_ERROR("Sensor data not received");
                return false;
            }
            
            counter = 0;
            data_to_record = request.data_to_record;

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
            ROS_INFO("Recording Done!");
            return true;
        }

        void timerCallback(const ros::TimerEvent&)
        {

            if(data_to_record==0)
                return;
            
            x_pos_arr[counter] = x_pos;
            y_pos_arr[counter] = y_pos;
            z_pos_arr[counter] = z_pos;
            x_rot_arr[counter] = x_rot;
            y_rot_arr[counter] = y_rot;
            z_rot_arr[counter] = z_rot;
            w_rot_arr[counter] = w_rot;


            data_to_record = data_to_record-1;
            counter = counter+1;
            if(data_to_record==0)
            {
                std::string full_file_path;
                full_file_path = file_path + file_name;
                std::fstream fout;
                fout.open(full_file_path, std::ios::out | std::ios::app);
                for(int i=0; i<counter; i++){
                    fout<< x_pos_arr[i] << ", "<< y_pos_arr[i] << ", "<< z_pos_arr[i] << ", ";
                    fout<< x_rot_arr[i] << ", "<< y_rot_arr[i] << ", "<< z_rot_arr[i] << ", "<< w_rot_arr[i] <<"\n";
                }

                fout.close();

                recording_done = true;
                // ROS_INFO("Recording Ended");
                counter = 0;
            }
        }

};




int main(int argc, char **argv)
{
    ros::init(argc, argv, "vrpn_recorder_node");
    ros::NodeHandle nh;

    nh.param<std::string>("/vrpn_recorder/data_topic",mocap_topic, "/vrpn_client_node/robot/pose");
    nh.param<std::string>("/vrpn_recorder/data_file_path",data_file_path, "/recorded_data.csv");

    nh.getParam("/vrpn_recorder/record_duration",record_duration);
    nh.getParam("/vrpn_recorder/use_data_directly",use_data_directly);


    ros::Time::init();

    DataCollector data_collector_object;
    ros::AsyncSpinner spinner(4);
    spinner.start();
    
    ros::Subscriber sub_mocap = nh.subscribe(mocap_topic, 1000, &DataCollector::forceCallback, &data_collector_object);

    if(use_data_directly==false){   // Use a timer to sync recording
        ros::Timer collect_data_timer = nh.createTimer(ros::Duration(record_duration), &DataCollector::timerCallback,  &data_collector_object);
    }

    ros::ServiceServer srv_record_data = nh.advertiseService("collect_data_service", &DataCollector::activateTimerServiceCallback, &data_collector_object);

    ros::waitForShutdown();


    return 0;
}