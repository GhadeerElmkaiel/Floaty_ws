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
// #include <eigen3>

#include <fcntl.h>
#include <stdio.h>
#include <unistd.h>
#include <math.h>
#include <cmath>
 
#ifdef _WIN32
#define O_NOCTTY 0
#else
#include <termios.h>
#endif

#include<unistd.h>
#include <fstream>


#define MOTOR_STOP_SPEED 4000

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
 
  ROS_INFO("Maestro Controller Initialized");
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
  // ROS_INFO("Motor rotation command sent.");

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

  int target = MOTOR_STOP_SPEED;
  for(int i=0; i<6; i++){
    maestroSetTarget(i, target);
  }
  return;
}

void initMotor(int motor_id)
{
  maestroSetTarget(motor_id, MOTOR_STOP_SPEED); 
  return;
}


void moveMotor(int speed, int motor_id)
{
  maestroSetTarget(motor_id, speed);   
  return;
}

// motorsCallback
// Callback to rotate and stop the motors
void motorsCallback(const std_msgs::Int32::ConstPtr& msg)
{
  ros::param::set("rotated", false);
  int val = msg->data;
  // if(val<4000)
  //   val=4000;
  // else if(val>8000)
  //   val=8000;
  
  moveMotor(val, 5);

  // float EMax_rot, DPower_rot, KST_rot;
  // ros::param::get("EMax_rot", EMax_rot);
  // ros::param::get("DPower_rot", DPower_rot);
  // ros::param::get("KST_rot", KST_rot);

  float motor_rot;

  std::map<std::string,float> orientation;

  // ros::param::get("motor_rot", motor_rot);
  ros::param::get("orientation", orientation);
  // ROS_INFO("Got A Motor Command");

  std::fstream fout;
  fout.open("motors.csv", std::ios::out | std::ios::app);
  // fout<< val << ", "<< EMax_rot << ", "<< DPower_rot << ", "<< KST_rot <<"\n";
  fout<< val << ", "<< orientation["X"] << ", "<< orientation["y"] << ", "<< orientation["z"] << ", "<< orientation["w"] <<"\n";
  fout.close();

}

void showRotArray(float ( &arr )[3][3]){
  ROS_INFO("[[ %f, %f, %f],",arr[0][0], arr[0][1], arr[0][2]);
  ROS_INFO(" [ %f, %f, %f],",arr[1][0], arr[1][1], arr[1][2]);
  ROS_INFO(" [ %f, %f, %f]]",arr[2][0], arr[2][1], arr[2][2]);
  ROS_INFO("______________");
  return;
}

void multArrayByScalar(float arr[3][3], float scal){
  for(int i =0; i<3; i++){
    for(int j=0; j<3; j++){
      arr[i][j]=arr[i][j]*scal;
    }
  }
  return;
}

void addTwoArrays(float arr[3][3], float arr2[3][3]){
  for(int i =0; i<3; i++){
    for(int j=0; j<3; j++){
      arr[i][j]=arr[i][j] + arr2[i][j];
    }
  }
  return;
}

void multArrayByArray(float arr[3][3], float arr2[3][3], float rslt[3][3]){
    
  for (int i = 0; i < 3; i++) {
    for (int j = 0; j < 3; j++) {
      rslt[i][j] = 0;

      for (int k = 0; k < 3; k++) {
        rslt[i][j] += arr[i][k] * arr2[k][j];
      }
    }
  }
  return;
}

void skewSymMatrix(float vec[3], float arr[3][3])
{
  arr[0][0] = 0;
  arr[0][1] = -vec[2];
  arr[0][2] = vec[1];

  arr[1][0] = vec[2];
  arr[1][1] = 0;
  arr[1][2] = -vec[0];

  arr[2][0] = -vec[1];
  arr[2][1] = vec[0];
  arr[2][2] = 0;

  return;
}


void rotationMatrixFromQuat(float quat[4], float arr[3][3]){
  for(int i =0; i<3; i++)
  {
    for(int j =0; j<3; j++)
    {
      if(i==j)
        arr[i][j]=1;
      else
        arr[i][j]=0;
    }
      
  }
  float skew1[3][3], skew2[3][3], res[3][3];

  float vec[3] = {quat[0], quat[1], quat[2]};
  skewSymMatrix(vec, skew1);
  skewSymMatrix(vec, skew2);
  multArrayByScalar(skew1, 2*quat[3]);
  addTwoArrays(arr, skew1);
  multArrayByArray(skew2, skew2, res);
  multArrayByScalar(res, 2);
  addTwoArrays(arr, res);

  return;
}



void mocapMotorCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
  float new_rot;
  std::map<std::string,float> orientation, position;


  position["x"] = msg->pose.position.x;
  position["y"] = msg->pose.position.y;
  position["z"] = msg->pose.position.z;

  orientation["x"] = msg->pose.orientation.x;
  orientation["y"] = msg->pose.orientation.y;
  orientation["z"] = msg->pose.orientation.z;
  orientation["w"] = msg->pose.orientation.w;

  // new_rot = msg->pose.orientation.z;
  // ros::param::set("motor_rot", new_rot);
  ros::param::set("orientation", orientation);
  // ros::param::set("position", position);
  // ROS_INFO("Optitrack");
  
}

 
int main(int argc, char **argv)
{

  ros::init(argc, argv, "listener");
  ros::NodeHandle n;

  ros::Time::init();

  int fd;
  // Init motors
  initMaestro();
  initMotor(5);
  
  ros::param::get("maestro_fd", fd);


  ros::param::set("last_rotation", 0.0);

  
  ros::Subscriber sub_motor = n.subscribe("motor", 1000, motorsCallback);
  ros::Subscriber sub_mocapmotor = n.subscribe("/vrpn_client_node/motor/pose", 1000, mocapMotorCallback);

  

  ros::spin();

  return 0;
  close(fd);
}