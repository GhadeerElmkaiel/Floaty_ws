// Uses POSIX functions to send and receive data from a Maestro.
// NOTE: The Maestro's serial mode must be set to "USB Dual Port".
// NOTE: You must change the 'const char * device' line below.
 
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
int maestroGetPosition(int fd, unsigned char channel)
{
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
int maestroSetTarget(int fd, unsigned char channel, unsigned short target)
{
  unsigned char command[] = {0x84, channel, target & 0x7F, target >> 7 & 0x7F};
  if (write(fd, command, sizeof(command)) == -1)
  {
    perror("error writing");
    return -1;
  }
  return 0;
}



void stopMotors(int fd)
{
  int target = 4000;
  maestroSetTarget(fd, 0, target);
  maestroSetTarget(fd, 1, target);
  maestroSetTarget(fd, 2, target);
  maestroSetTarget(fd, 3, target);
  maestroSetTarget(fd, 4, target);
  maestroSetTarget(fd, 5, target);    
  return;
}
 
int main()
{
  // Open the Maestro's virtual COM port.
//   const char * device = "\\\\.\\COM6";  // Windows
  const char * device = "/dev/ttyACM0";  // Linux
  const int min_speed = 4220;
  //const char * device = "/dev/cu.usbmodem00034567";  // Mac OS X
  int fd = open(device, O_RDWR | O_NOCTTY);
  printf("O_RDWR %d , O_NOCTTY %d \n", O_RDWR, O_NOCTTY);
  printf("fd %d \n", fd);
  if (fd == -1)
  {
    perror(device);
    return 1;
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
 
  // int position = maestroGetPosition(fd, 0);
  // printf("Current position is %d.\n", position);

  // Init motos
  stopMotors(fd);    

  int target = min_speed;
  printf("Setting target to %d (%d us).\n", target, target/4);
  maestroSetTarget(fd, 1, target);
  maestroSetTarget(fd, 2, target);
  usleep(3000 * 1000);

  stopMotors(fd);
  printf("fd %d.\n", fd);
  close(fd);
  return 0;
}