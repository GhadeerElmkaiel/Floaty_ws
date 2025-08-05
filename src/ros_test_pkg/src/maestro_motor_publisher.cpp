// ROS
#include "ros/ros.h"
#include "std_msgs/Int32.h"

#include <math.h>

//


int main(int argc, char **argv)
{
    ros::init(argc, argv, "motor_publisher");
    ros::NodeHandle n;

    // Create an array for the motor's signal 
    int arr_size;
    float freq;
    arr_size=16001;
    freq = 1000;

    // Create a square signal for the motor
    int motor_vals[arr_size];
    for(int i=0; i<arr_size; i++){

        motor_vals[i] = 4000 + 2000*(1-cos(i*0.004*3.1416));
        // _______________________________________
        // Square signal   _|-|_|-|_|-|_|-|_|-|
        // int temp_val;
        // temp_val = int(i/freq);

        // if(temp_val%4 ==0)
        // motor_vals[i]=4000;
        // else if(temp_val%4 ==1)
        // motor_vals[i]=4018;
        // else if(temp_val%4 ==2)
        // motor_vals[i]=4035;
        // else if(temp_val%4 ==3)
        // motor_vals[i]=4071;

    }

    ros::Publisher motor_pub = n.advertise<std_msgs::Int32>("motor", freq);
    // ros::Publisher kst_pub = n.advertise<std_msgs::Int32>("kst", freq);
    // ros::Publisher dpower_pub = n.advertise<std_msgs::Int32>("dpower", freq);
    ros::Rate loop_rate(freq);

    int count = 0;
    while (count<arr_size)
    {
        std_msgs::Int32 msg;
        msg.data = motor_vals[count];
        // ROS_INFO("%d", msg.data);
        motor_pub.publish(msg);
        // dpower_pub.publish(msg);
        // kst_pub.publish(msg);
        
        ros::spinOnce();

        loop_rate.sleep();
        count++;
    }

    return 0;
}