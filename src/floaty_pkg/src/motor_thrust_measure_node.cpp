// ROS
#include "ros/ros.h"

// ROS service messages
#include<floaty_msgs/maestro_srv.h>
#include<floaty_msgs/record_data_srv.h>
#include<floaty_msgs/motor_thrust_measure_srv.h>

class MotorThrustMeasure
{
    private:
        ros::ServiceClient motors_speed_client;
        ros::ServiceClient collect_data_client;
    public:
        MotorThrustMeasure(ros::ServiceClient collect_client, ros::ServiceClient motors_client)
        {
            motors_speed_client = motors_client;
            collect_data_client = collect_client;
        }


        bool motor_thrust_measure(floaty_msgs::motor_thrust_measure_srv::Request& request, floaty_msgs::motor_thrust_measure_srv::Response& response){
            ROS_INFO("Got request to start motor thrust measurement");
            int motor_id = request.motor_id;
            int start_speed = request.start_speed;
            int end_speed = request.end_speed;
            int increment = request.increment;
            int measurments_per_speed = request.measurments_per_speed;
            if(start_speed<0){
                ROS_ERROR("Starting speed is less than 0%");
                return false;
            }
            if(end_speed>100){
                ROS_ERROR("End speed is more than 100%");
                return false;
            }
            if(increment>20){
                ROS_ERROR("Increment is more than 20%");
                return false;
            }
            if(motor_id>5||motor_id<0){
                ROS_ERROR("Motor id is not correct");
                return false;
            }

            floaty_msgs::maestro_srv motor_srv;
            motor_srv.request.speeds = {4000, 4000, 4000, 4000, 4000, 4000};
            // motor_srv.request.speeds[motor_id] = 4000+start_speed*40;


            floaty_msgs::record_data_srv record_srv;
            record_srv.request.data_to_record = measurments_per_speed;

            for(int i=start_speed; i<=end_speed; i+=increment){
                // Setting the motor speed
                motor_srv.request.speeds[motor_id] = 4000+(start_speed+i)*40;
                if(!motors_speed_client.call(motor_srv)){
                    ROS_ERROR("Motor Service Not Working");
                }
                ROS_INFO("New motor speed set");
                ros::Duration(1).sleep();

                if(!collect_data_client.call(record_srv)){
                    ROS_ERROR("Recording Data Service Not Working");
                }
                ROS_INFO("Started the recording for new speed");
                ros::Duration(0.1*measurments_per_speed+1).sleep();
            }

            motor_srv.request.speeds = {4000, 4000, 4000, 4000, 4000, 4000};
            motors_speed_client.call(motor_srv);

            return true;
        }

};


int main(int argc, char **argv)
{

    ros::init(argc, argv, "motor_thrust_measure_node");
    ros::NodeHandle nh;


    ros::ServiceClient motors_speed_client = nh.serviceClient<floaty_msgs::maestro_srv>("maestro_control/move_motors_service");
    ros::ServiceClient collect_data_client = nh.serviceClient<floaty_msgs::record_data_srv>("data_recorder/collect_data_service");

    MotorThrustMeasure motor_thrust_measure_object(collect_data_client, motors_speed_client);

    ros::ServiceServer srv_motor_thrust_measure = nh.advertiseService("motor_thrust_measure_service", &MotorThrustMeasure::motor_thrust_measure, &motor_thrust_measure_object);


    ros::spin();

    return 0;
}