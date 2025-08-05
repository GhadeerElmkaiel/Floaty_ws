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

#include <algorithm> // for sort function


#include<unistd.h>
#include <fstream>

// A function to get the median of an array. It is used for the ATI sensor data
float median(float * arr, int n) {
  std::sort(arr, arr + n); // sort the array

  if (n % 2 == 0) {
    // if the array has an even number of elements, take the average of the middle two
    return (float)(arr[n / 2 - 1] + arr[n / 2]) / 2.0;
  } else {
    // otherwise, return the middle element
    return (float)arr[n / 2];
  }
}


std::string data_topic, mocap_topic; //, data_file_path;
double record_duration;
bool use_median_value_for_ATI;
int num_of_values_to_use_for_ATI;

class DataCollector
{
    private:
        float x_pos, y_pos, z_pos, x_force, y_force, z_force, x_torque, y_torque, z_torque;
        int data_to_record;
        int num_of_data_points;
        // std::string file_path;
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
        float* last_n_x_force = new float[num_of_values_to_use_for_ATI];
        float* last_n_y_force = new float[num_of_values_to_use_for_ATI];
        float* last_n_z_force = new float[num_of_values_to_use_for_ATI];
        float* last_n_x_torque = new float[num_of_values_to_use_for_ATI];
        float* last_n_y_torque = new float[num_of_values_to_use_for_ATI];
        float* last_n_z_torque = new float[num_of_values_to_use_for_ATI];
        int last_iter;
    public:
    // Constructor function
    DataCollector() : x_pos(0), y_pos(0), z_pos(0), x_force(0), y_force(0), z_force(0), x_torque(0), y_torque(0), z_torque(0), data_to_record(0), num_of_data_points(0), file_name("recorded_data.csv"), recording_done(false), last_iter(0) { }  // , file_path(data_file_path)

        void forceCallback(const geometry_msgs::WrenchStamped::ConstPtr& msg)
        {
            x_force = msg->wrench.force.x;
            y_force = msg->wrench.force.y;
            z_force = msg->wrench.force.z;
            x_torque = msg->wrench.torque.x;
            y_torque = msg->wrench.torque.y;
            z_torque = msg->wrench.torque.z;
            last_n_x_force[last_iter] = x_force;
            last_n_y_force[last_iter] = y_force;
            last_n_z_force[last_iter] = z_force;
            last_n_x_torque[last_iter] = x_torque;
            last_n_y_torque[last_iter] = y_torque;
            last_n_z_torque[last_iter] = z_torque;
            last_iter = (last_iter+1)%num_of_values_to_use_for_ATI;
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

            int iter = num_of_data_points - data_to_record;
            x_pos_arr[iter] = x_pos;
            y_pos_arr[iter] = y_pos;
            z_pos_arr[iter] = z_pos;
            if(use_median_value_for_ATI){
                x_force_arr[iter] = median(last_n_x_force, num_of_values_to_use_for_ATI);
                y_force_arr[iter] = median(last_n_y_force, num_of_values_to_use_for_ATI);
                z_force_arr[iter] = median(last_n_z_force, num_of_values_to_use_for_ATI);
                x_torque_arr[iter] = median(last_n_x_torque, num_of_values_to_use_for_ATI);
                y_torque_arr[iter] = median(last_n_y_torque, num_of_values_to_use_for_ATI);
                z_torque_arr[iter] = median(last_n_z_torque, num_of_values_to_use_for_ATI);
            }
            else{
                x_force_arr[iter] = x_force;
                y_force_arr[iter] = y_force;
                z_force_arr[iter] = z_force;
                x_torque_arr[iter] = x_torque;
                y_torque_arr[iter] = y_torque;
                z_torque_arr[iter] = z_torque;
            }

            data_to_record = data_to_record-1;
            if(data_to_record==0)
            {
                std::fstream fout;
                // fout.open("motors.csv", std::ios::out | std::ios::app);
                // fout.open(file_path+file_name, std::ios::out | std::ios::app);
                fout.open(file_name, std::ios::out | std::ios::app);
                fout<< "x_pos" << ", "<< "y_pos" << ", " << "z_pos" << ", ";
                fout<< "x_force" << ", "<< "y_force" << ", "<< "z_force"  << ", ";
                fout<< "x_torque" << ", "<< "y_torque" << ", "<< "z_torque" << "\n";

                for(int i=0; i<num_of_data_points; i++){
                    fout<< x_pos_arr[i] << ", "<< y_pos_arr[i] << ", "<< z_pos_arr[i] <<", ";
                    fout<< x_force_arr[i] << ", "<< y_force_arr[i] << ", "<< z_force_arr[i]  <<", ";
                    fout<< x_torque_arr[i] << ", "<< y_torque_arr[i] << ", "<< z_torque_arr[i] << "\n";
                }
                // fout<< x_pos << ", "<< y_pos << ", "<< z_force <<"\n";

                fout.close();
                recording_done = true;
                ROS_INFO("Recording Ended");
            }
        }

};


int main(int argc, char **argv)
{

    ros::init(argc, argv, "data_recorder_node");
    ros::NodeHandle nh;

    nh.param<std::string>("/ft_sensor/pub_topic",data_topic, "netft_data");
    nh.param<std::string>("/NatNet/data_topic",mocap_topic, "/Optitrack/Floaty");
    // nh.param<std::string>("/data_recorder/data_file_path",data_file_path, "/home/gelmkaiel/Floaty/ws/src/floaty_pkg/data/online_learning/current_experiment/");

    data_topic = "/ft_sensor/" + data_topic;

    nh.getParam("/data_recorder/use_median_value_for_ATI",use_median_value_for_ATI);
    nh.getParam("/data_recorder/record_period",record_duration);
    nh.getParam("/data_recorder/num_of_values_to_use_for_ATI",num_of_values_to_use_for_ATI);


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