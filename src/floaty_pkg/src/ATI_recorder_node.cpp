/*********************************************************************
 * This code is a part of Floaty project
 * 
 *********************************************************************/

/**
 * Simple stand-alone ROS node that record data from different topics
 * and savethem in a file
 */

#include "ros/ros.h"

#include "geometry_msgs/WrenchStamped.h"

#include<floaty_msgs/record_data_srv.h>


// #include "netft_rdt_driver/netft_rdt_driver.h"
// #include "netft_rdt_driver/netft_rdt_bias.h"

#include<unistd.h>
#include <fstream>

std::string data_topic;
// std::string data_file_path;
// int num_data_to_record;
double record_duration;
bool use_ati_data_directly;

class DataCollector
{
    private:
        float x_force_arr [40000];
        float y_force_arr [40000];
        float z_force_arr [40000];
        float x_torque_arr [40000];
        float y_torque_arr [40000];
        float z_torque_arr [40000];
        float x_force, y_force, z_force, x_torque, y_torque, z_torque;
        int data_to_record;
        std::string file_name;
        // std::string file_path;
        bool recording_done;
        int counter;
        bool receiver_sensor_data;
        bool use_ati_data;
    public:
    // Constructor function
    DataCollector() : x_force(0), y_force(0), z_force(0), x_torque(0), y_torque(0), z_torque(0), data_to_record(0), file_name("data.csv"), recording_done(false), counter(0), receiver_sensor_data(false), use_ati_data(use_ati_data_directly) { }     // , file_path(data_file_path)

        void forceCallback(const geometry_msgs::WrenchStamped::ConstPtr& msg)
        {
            receiver_sensor_data = true;
            if(use_ati_data){               // Recording directly when recieving data from sensor
                if(data_to_record==0)
                    return;
                // receiver_sensor_data = true;
                x_force_arr[counter] = msg->wrench.force.x;
                y_force_arr[counter] = msg->wrench.force.y;
                z_force_arr[counter] = msg->wrench.force.z;
                x_torque_arr[counter] = msg->wrench.torque.x;
                y_torque_arr[counter] = msg->wrench.torque.y;
                z_torque_arr[counter] = msg->wrench.torque.z;

                data_to_record = data_to_record-1;
                counter = counter+1;
                if(data_to_record==0)
                {
                    std::string full_file_path;
                    // full_file_path = file_path + file_name;
                    full_file_path = file_name;
                    std::fstream fout;
                    fout.open(full_file_path, std::ios::out | std::ios::app);
                    for(int i=0; i<counter; i++){
                        fout<< x_force_arr[i] << ", "<< y_force_arr[i] << ", "<< z_force_arr[i] << ", ";
                        fout<< x_torque_arr[i] << ", "<< y_torque_arr[i] << ", "<< z_torque_arr[i] <<"\n";
                    }

                    fout.close();

                    recording_done = true;
                    ROS_INFO("Recording Ended");
                    counter = 0;
                }
            }
            else{
                // receiver_sensor_data = true;
                x_force = msg->wrench.force.x;
                y_force = msg->wrench.force.y;
                z_force = msg->wrench.force.z;
                x_torque = msg->wrench.torque.x;
                y_torque = msg->wrench.torque.y;
                z_torque = msg->wrench.torque.z;
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
            // ROS_INFO("Data To record ");
            if(data_to_record==0)
                return;
            
            // ROS_INFO("Recording a datapoint");
            x_force_arr[counter] = x_force;
            y_force_arr[counter] = y_force;
            z_force_arr[counter] = z_force;
            x_torque_arr[counter] = x_torque;
            y_torque_arr[counter] = y_torque;
            z_torque_arr[counter] = z_torque;


            data_to_record = data_to_record-1;
            counter = counter+1;
            if(data_to_record==0)
            {
                ROS_INFO("Writing to file");
                
                std::string full_file_path;
                // full_file_path = file_path + file_name;
                full_file_path = file_name;
                std::fstream fout;
                fout.open(full_file_path, std::ios::out | std::ios::app);
                for(int i=0; i<counter; i++){
                    fout<< x_force_arr[i] << ", "<< y_force_arr[i] << ", "<< z_force_arr[i] << ", ";
                    fout<< x_torque_arr[i] << ", "<< y_torque_arr[i] << ", "<< z_torque_arr[i] <<"\n";
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

    ros::init(argc, argv, "ATI_recorder_node");
    ros::NodeHandle nh;

    // nh.param<std::string>("/ATI_recorder/data_topic",data_topic, "/ft_sensor/netft_data");
    // nh.param<std::string>("/ATI_recorder/data_file_path",data_file_path, "/recorded_data.csv");

    nh.param<std::string>("/ft_sensor/pub_topic",data_topic, "netft_data");
    data_topic = "/ft_sensor/" + data_topic;

    // nh.getParam(nh.getNamespace() + "/data_to_record",num_data_to_record);
    // nh.getParam(nh.getNamespace() + "/record_duration",record_duration);
    nh.getParam("/ft_sensor/record_duration",record_duration);
    nh.getParam("/ft_sensor/use_ati_data_directly",use_ati_data_directly);

    // Resetting the ATI sensor bias
    // netft_rdt_driver::String_cmd package;
    // package.request.cmd == "bias";
    // ros::service::call("/ft_sensor/bias_cmd", package);

    // ROS_INFO(data_file_name);

    ros::Time::init();

    DataCollector data_collector_object;
    ros::AsyncSpinner spinner(4);
    spinner.start();
    
    ros::Subscriber sub_force = nh.subscribe(data_topic, 1000, &DataCollector::forceCallback, &data_collector_object);

    // if(use_ati_data_directly==false){   // Use a timer to sync recording
    //     ROS_INFO("Activating recording timer");
    //     ros::Timer collect_data_timer = nh.createTimer(ros::Duration(record_duration), &DataCollector::timerCallback,  &data_collector_object);
    // }
    ros::Timer collect_data_timer = nh.createTimer(ros::Duration(record_duration), &DataCollector::timerCallback,  &data_collector_object);


    ros::ServiceServer srv_record_data = nh.advertiseService("collect_data_service", &DataCollector::activateTimerServiceCallback, &data_collector_object);

    ros::waitForShutdown();

    return 0;
}