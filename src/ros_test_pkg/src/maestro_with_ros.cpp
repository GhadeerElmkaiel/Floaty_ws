// ROS
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Int32.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Point.h"
#include "geometry_msgs/Quaternion.h"
//
 
#include <ctime>
#include <time.h>

#include <fcntl.h>
#include <stdio.h>
#include <unistd.h>
 
#ifdef _WIN32
#define O_NOCTTY 0
#else
#include <termios.h>
#endif

#include<unistd.h>
 
// Gets the position of a Maestro channel.
// See the "Serial Servo Commands" section of the user's guide.
int maestroGetPosition(unsigned char channel)
{
  int fd;
  ros::param::get("/maestro_fd", fd);

  unsigned char command[] = {0x90, channel};
  if(write(fd, command, sizeof(command)) == -1)
  {
    perror("error writing");
    return -1;
  }
 
  unsigned char response[2];
  if(read(fd,response,2) != 2)
  {
    perror("error reading");
    return -1;
  }
 
  return response[0] + 256*response[1];
}
 
// Sets the target of a Maestro channel.
// See the "Serial Servo Commands" section of the user's guide.
// The units of 'target' are quarter-microseconds.
int maestroSetTarget(unsigned char channel, unsigned short target)
{
  int fd;
  ros::param::get("/maestro_fd", fd);
  ros::Time time_now = ros::Time::now();
  // std::time_t ms = std::time();
  ROS_INFO("Motor rotation command sent.");

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
  const char * device = "/dev/ttyACM0";  // Linux
//   ros::param::set("maestro_device", device);
  int fd = open(device, O_RDWR | O_NOCTTY);

  if (fd == -1)
  {
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
  ros::param::set("maestro_fd", fd);
}

void stopMotors()
{
  int fd;
  ros::param::get("maestro_fd", fd);

  int target = 4000;
  for(int i=0; i<6; i++){
    maestroSetTarget(i, target);
  }
  return;
}

void moveMotor(int speed, int motor_id)
{
  ROS_INFO("Running motor %d at speed %d", motor_id, speed);
  maestroSetTarget(motor_id, speed);   
  return;
}

void chatterCallback(const std_msgs::String::ConstPtr& msg)
{
  ROS_INFO("I heard: [%s]", msg->data.c_str());
}

// motorsCallback
// Callback to rotate and stop the motors
void motorsCallback(const std_msgs::Int32::ConstPtr& msg)
{
  ros::param::set("rotated", false);
  int val = msg->data;
  if(val<4000)
    val=4000;
  else if(val>8000)
    val=8000;
  
  moveMotor(val, 5);


}

// mocapCallback
// Callback for the motion capture system
void mocapCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
  bool rotated, init;

  ros::param::get("rotated", rotated);
  ros::param::get("init", init);
  if(!init)
  {
    float new_rot;
    new_rot = msg->pose.orientation.z;
    ros::param::set("last_rotation", new_rot);
    ROS_INFO("Init mocap Callback");
    ROS_INFO("last_rotation was: %d", new_rot);
    ros::param::set("init", true);
  }
  if(rotated)
  {
    return;
  }
  float last_rot, new_rot, eps;
  new_rot = msg->pose.orientation.z;

  ros::param::get("last_rotation", last_rot);
  ros::param::get("epsilon", eps);

  ros::param::set("last_rotation", new_rot);
  
  bool detect_rot;
  detect_rot = (last_rot-new_rot>eps || last_rot-new_rot<-eps);
  if(detect_rot)
  {
    ros::param::set("rotated", true);
    ros::Time time_now = ros::Time::now();
    // std::time_t ms = std::time();
    ROS_INFO("Detected motor Rotation.");
  }
}

 
int main(int argc, char **argv)
{

  ros::init(argc, argv, "listener");
  ros::NodeHandle n;

  ros::Time::init();

  int fd;
  // Init motos
  initMaestro();
  
  ros::param::get("maestro_fd", fd);


  // ROS

  ros::param::set("last_rotation", 0.0);
  ros::param::set("epsilon", 0.002);
  ros::param::set("rotated", false);
  ros::param::set("init", false);
  
  ros::Subscriber sub = n.subscribe("chatter", 1000, chatterCallback);

  ros::Subscriber sub_motor = n.subscribe("motor", 1000, motorsCallback);

  ros::Subscriber sub_mocap = n.subscribe("/vrpn_client_node/motor/pose", 1000, mocapCallback);


  ros::spin();

  return 0;
  close(fd);
}