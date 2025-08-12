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
//#define MOTOR_MIN_ROT_SPEED 4170
#define MOTOR_MIN_ROT_SPEED 4230

int number_of_motors;
int fd;
int fd2;
// int last_speeds[6] = {MOTOR_STOP_SPEED, MOTOR_STOP_SPEED, MOTOR_STOP_SPEED, MOTOR_STOP_SPEED, MOTOR_STOP_SPEED, MOTOR_STOP_SPEED};
int last_speeds[7] = {MOTOR_STOP_SPEED, MOTOR_STOP_SPEED, MOTOR_STOP_SPEED, MOTOR_STOP_SPEED, MOTOR_STOP_SPEED, MOTOR_STOP_SPEED, MOTOR_STOP_SPEED};
int ramp_speed_divisions;
float ramp_time;
int max_motor_speed;

int vendorId = 0x1ffb;
int productId = 0x0089;

// const char* serialNumber1 = "00384346"; // Serial number for device 1
// const char* serialNumber2 = "00444816"; // Serial number for device 2

std::string serialNumber1 = "00384346"; // Serial number for device 1
std::string serialNumber2 = "00444816"; // Serial number for device 2

// Sets the target of a Maestro channel.
// See the "Serial Servo Commands" section of the user's guide.
// The units of 'target' are quarter-microseconds.
int maestroSetTarget(unsigned char channel, unsigned short target)
{
  // ROS_INFO("Setting motor value");
  // int fd;
  // ros::param::get(name_space + "/maestro_fd", fd);

  // unsigned char command[] = {0x84, channel, target & 0x7F, target >> 7 & 0x7F};
  // if (write(fd, command, sizeof(command)) == -1)
  // {
  //   perror("error writing");
  //   return -1;
  // }
  // return 0;

  if(channel<6){
    unsigned char command[] = {0x84, channel, target & 0x7F, target >> 7 & 0x7F};
    if (write(fd, command, sizeof(command)) == -1)
    {
      perror("error writing");
      return -1;
    }
    return 0;
  }
  else{
    unsigned char adjustedChannel = channel - 6;
    unsigned char command[] = {0x84, adjustedChannel, target & 0x7F, target >> 7 & 0x7F};
    if (write(fd2, command, sizeof(command)) == -1)
    {
      perror("error writing");
      return -1;
    }
    return 0;
  }
}

void initMaestro()
{

  const char * device = "/dev/ttyACM0";
  // const char * device = "/dev/ttyACM1";
  const char * device2 = "/dev/ttyACM2";
  // const char * device2 = "/dev/ttyACM3";

  // std::string ttyACMPath1 = findTtyACMBySerial(serialNumber1);
  // std::string ttyACMPath2 = findTtyACMBySerial(serialNumber2);

  // const char* device = ttyACMPath1.c_str();
  // const char* device2 = ttyACMPath2.c_str();

//   ros::param::set("maestro_device", device);
  // int fd = open(device, O_RDWR | O_NOCTTY);
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

  fd2 = open(device2, O_RDWR | O_NOCTTY);

  if (fd2 == -1)
  {
    ROS_ERROR("Maestro 2 Not Found!");
    perror(device2);
    return;
  }

 
#ifdef _WIN32
  _setmode(fd, _O_BINARY);
#else
  struct termios options2;
  tcgetattr(fd2, &options2);
  options2.c_iflag &= ~(INLCR | IGNCR | ICRNL | IXON | IXOFF);
  options2.c_oflag &= ~(ONLCR | OCRNL);
  options2.c_lflag &= ~(ECHO | ECHONL | ICANON | ISIG | IEXTEN);
  tcsetattr(fd2, TCSANOW, &options2);

#endif

  // ros::param::set(name_space + "/maestro_fd", fd);
  for(int i=0; i<number_of_motors; i++){
    maestroSetTarget(i, MOTOR_STOP_SPEED);
  }
  
  ROS_INFO("fd value: %d", fd);
  ROS_INFO("fd2 value: %d", fd2);
  ROS_INFO("Maestro ready");

}


bool stopMotorsCallback(floaty_msgs::empty_srv::Request& request, floaty_msgs::empty_srv::Response& response)
{
  ROS_INFO("Stopping the motors!");
  int target = MOTOR_STOP_SPEED;
  for(int i=0; i<number_of_motors; i++){
    maestroSetTarget(i, target);
    last_speeds[i]=target;
  }
  // ROS_INFO("Finished stopping the motors!");
  
  return true;
}

bool moveMotorsSameSpeedCallback(floaty_msgs::int_srv::Request& request, floaty_msgs::int_srv::Response& response)
{
  int target = request.data;
  if(target>=0 && target<=100){
    target=40*(target+100);
  }
  if(target>max_motor_speed){
    target=max_motor_speed;
  }
  for(int i=0; i<number_of_motors; i++){
    maestroSetTarget(i, target);
    last_speeds[i]=target;
  }
  return true;
}

bool rampMotorsSameSpeedCallback(floaty_msgs::int_srv::Request& request, floaty_msgs::int_srv::Response& response)
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
    for(int i=0; i<number_of_motors; i++){
      // Getting the initial speed for motor i and calculate the delta
      int initial_speed = last_speeds[i];
      int delta_speed = (int)((target-initial_speed)/ramp_speed_divisions);
      int new_speed = initial_speed+j*delta_speed;
      maestroSetTarget(i, new_speed);
    }
    ros::Duration(wait_time).sleep();
  }

  for(int i=0; i<number_of_motors; i++){
    maestroSetTarget(i, target);
    last_speeds[i]=target;
  }
  return true;
}

bool minRotateCallback(floaty_msgs::empty_srv::Request& request, floaty_msgs::empty_srv::Response& response)
{
  int target = MOTOR_MIN_ROT_SPEED;
  for(int i=0; i<number_of_motors; i++){
    maestroSetTarget(i, target);
    last_speeds[i]=target;
  }
  return true;
}


bool moveMotorsCallback(floaty_msgs::maestro_srv::Request& request, floaty_msgs::maestro_srv::Response& response)
{
  // int ids = request.motors;
  int target;
  int size_ = sizeof(request.speeds)/sizeof(request.speeds[0]);
  for(int i=0; i<size_; i++){
    target = request.speeds[i];
    if(target>=0 && target<=100){
      target=40*(target+100);
    }
    if(target>max_motor_speed){
      target=max_motor_speed;
    }
    ROS_INFO("Speed for motor [%d] is: [%d]", i, request.speeds[i]);
    maestroSetTarget(i, target);
    last_speeds[i]=target;
  }
  return true;
}


bool rampMotorsCallback(floaty_msgs::maestro_srv::Request& request, floaty_msgs::maestro_srv::Response& response)
{
  float wait_time = ramp_time/ramp_speed_divisions;
  // int size_ = sizeof(request.speeds)/sizeof(request.speeds[0]);
  int size_ = number_of_motors;
  int target;
  ROS_INFO("Num of motors (size_) = [%d]",size_);

  for(int j=0; j<ramp_speed_divisions-1; j++){
    for(int i=0; i<size_; i++){
      // Getting the initial speed for motor i and calculate the delta
      int initial_speed = last_speeds[i];
      target = request.speeds[i];
      if(target>=0 && target<=100){
        target=40*(target+100);
      }
      int delta_speed = (int)((target-initial_speed)/ramp_speed_divisions);
      int new_speed = initial_speed+j*delta_speed;
      if(new_speed>max_motor_speed){
        new_speed=max_motor_speed;
      }
      maestroSetTarget(i, new_speed);
    }
    ros::Duration(wait_time).sleep();
  }
  // Sending the correct target speed (To avoid rounding issues)
  for(int i=0; i<size_; i++){
    target = request.speeds[i];
    if(target>=0 && target<=100){
      target=40*(target+100);
    }
    if(target>max_motor_speed){
      target=max_motor_speed;
    }
    ROS_INFO("Speed for motor [%d] is: [%d]", i, target);
    maestroSetTarget(i, target);
    last_speeds[i]=target;
  }

  return true;
}


int main(int argc, char **argv)
{

    ros::init(argc, argv, "maestro_node");
    ros::NodeHandle nh;

    nh.getParam("/maestro_control/number_of_motors",number_of_motors);
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
    
    ros::ServiceServer srv_move_motors = nh.advertiseService("move_motors_service", moveMotorsCallback);
    ros::ServiceServer srv_move_motors_same_speed = nh.advertiseService("move_motors_same_speed_service", moveMotorsSameSpeedCallback);
    ros::ServiceServer srv_ramp_motors = nh.advertiseService("ramp_motors_service", rampMotorsCallback);
    ros::ServiceServer srv_ramp_motors_same_speed = nh.advertiseService("ramp_motors_same_speed_service", rampMotorsSameSpeedCallback);
    ros::ServiceServer srv_stop_motors = nh.advertiseService("stop_motors_service", stopMotorsCallback);
    ros::ServiceServer srv_min_rot_motors = nh.advertiseService("min_rot_service", minRotateCallback);

    
    ros::spin();

    return 0;
}