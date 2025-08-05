// ROS
#include "ros/ros.h"
#include<floaty_msgs/maestro_srv.h>
#include<floaty_msgs/empty_srv.h>
#include<floaty_msgs/int_srv.h>

#include <fcntl.h>
#include <stdio.h>
#include <unistd.h>
#include <libusb.h>

#ifdef _WIN32
#define O_NOCTTY 0
#else
#include <termios.h>
#endif

// #include<unistd.h>
#include <fstream>
#include <string>
#include <boost/filesystem.hpp>

// #include <dirent.h>
// #include <cstring>
// #include <cstdio>
// #include <cstdlib>
// #include <iostream>
// #include <glob.h>

namespace fs = boost::filesystem;

// #include <libudev.h>

#define MOTOR_STOP_SPEED 4000
#define MOTOR_MIN_ROT_SPEED 4170

int fd;

// int last_speeds[6] = {MOTOR_STOP_SPEED, MOTOR_STOP_SPEED, MOTOR_STOP_SPEED, MOTOR_STOP_SPEED, MOTOR_STOP_SPEED, MOTOR_STOP_SPEED};
int last_speed = MOTOR_STOP_SPEED;
int ramp_speed_divisions;
float ramp_time;
int max_motor_speed;

int vendorId = 0x1ffb;
int productId = 0x0089;


// Sets the target of a Maestro channel.
// See the "Serial Servo Commands" section of the user's guide.
// The units of 'target' are quarter-microseconds.
int maestroSetTarget(unsigned char channel, unsigned short target)
{

  unsigned char command[] = {0x84, channel, target & 0x7F, target >> 7 & 0x7F};
  if (write(fd, command, sizeof(command)) == -1)
  {
    perror("error writing");
    return -1;
  }
  return 0;
  
}

void initMaestro()
{

  const char * device = "/dev/ttyACM0";
  // const char * device = "/dev/ttyACM1";
  fd = open(device, O_RDWR | O_NOCTTY);

  if (fd == -1)
  {
    ROS_ERROR("Maestro Not Found!");
    perror(device);
    return;
  }

 
#ifdef _WIN32
  _setmode(fd, _O_BINARY);
#else
  struct termios options;
  tcgetattr(fd, &options);
  options.c_iflag &= ~(INLCR | IGNCR | ICRNL | IXON | IXOFF);
  options.c_oflag &= ~(ONLCR | OCRNL);
  options.c_lflag &= ~(ECHO | ECHONL | ICANON | ISIG | IEXTEN);
  tcsetattr(fd, TCSANOW, &options);

#endif

// ros::param::set(name_space + "/maestro_fd", fd);
maestroSetTarget(0, MOTOR_STOP_SPEED);


ROS_INFO("fd value: %d", fd);
ROS_INFO("Maestro Side wind ready");

}


bool stopSideMotorCallback(floaty_msgs::empty_srv::Request& request, floaty_msgs::empty_srv::Response& response)
{
  ROS_INFO("Stopping the motors!");
  int target = MOTOR_STOP_SPEED;
  maestroSetTarget(0, target);
  last_speed=target;
  
  // ROS_INFO("Finished stopping the motors!");
  
  return true;
}

bool rampSideMotorSpeedCallback(floaty_msgs::int_srv::Request& request, floaty_msgs::int_srv::Response& response)
{
  int target = request.data;
  float wait_time = ramp_time/ramp_speed_divisions;

  if(target>=0 && target<=100){
    target=40*(target+100);
  }
  if(target>max_motor_speed){
    target=max_motor_speed;
  }

  for(int j=0; j<ramp_speed_divisions-1; j++){
      // Getting the initial speed for motor i and calculate the delta
      int initial_speed = last_speed;
      int delta_speed = (int)((target-initial_speed)/ramp_speed_divisions);
      int new_speed = initial_speed+j*delta_speed;
      maestroSetTarget(0, new_speed);
    
    ros::Duration(wait_time).sleep();
  }

    maestroSetTarget(0, target);
    last_speed=target;
  
  return true;
}



int main(int argc, char **argv)
{

    ros::init(argc, argv, "maestro_side_wind_node");
    ros::NodeHandle nh;

    nh.getParam("/maestro_control/ramp_speed_divisions",ramp_speed_divisions);
    nh.getParam("/maestro_control/ramp_time",ramp_time);
    nh.getParam("/maestro_control/max_motor_speed",max_motor_speed);
    
    // Init motors
    initMaestro();

    if(ramp_time==0){
      ramp_time=0.8;
    }

    if(ramp_speed_divisions==0){
      ramp_speed_divisions=10;
    }

    ros::Time::init();
    
    ros::ServiceServer srv_ramp_side_motor_speed = nh.advertiseService("ramp_side_motor_speed_service", rampSideMotorSpeedCallback);
    ros::ServiceServer srv_stop_side_motor = nh.advertiseService("stop_side_motor_service", stopSideMotorCallback);

    
    ros::spin();

    return 0;
}