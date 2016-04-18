/*
  Title:  Header file for serial control of Qik motor controller
  Author: Rob Shiely
  Date:   4/12/2012
  TODOs:  Need functions to get current from controllers
*/
#ifndef _qik_h
#define _qik_h

#include <sys/types.h>
#include <sys/stat.h>
#include <sys/ioctl.h>
#include <limits.h>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdarg.h>
#include <time.h>
#include <errno.h>
#include <string.h>
#include "rnr/serdev.h"
#include "rnr/log.h"
/*
  BAUD rate to use for Qik motor controllers.
  This rate can be changed in Qik registers.
*/
#define BAUDRATE        38400

struct Qiks {
  int m_fdQiks;
  int m_fdQiks2; // DHP: for future
};

/*
  Function opens serial comunication with set port.
  (No actual communication with Qik controllers in this function)
  Returns: 0 on success and -1 on failure (0 or -1) 
*/
int QikOpenConnection(int *Descriptor); 


/*
  Function updates the speed of a single wheel.
  Needs to be passed: A speed (-250 < speed < 250), a file descriptor,
                      and a motor (0 = left, 1 = right)
  Returns: 0 on Success, -1 on incorrect speed, -2 on serial write fail.
*/
int QikUpdateMotorSpeeds(int Speed, int Descriptor, int Side);


/*
  Function changes how agressively each individual motor brakes.
  Needs to be passed: A brake rate (0 < Braking < 31: 31 = full brake),
                      a file descriptor, and a motor)
  Returns: 0 on Success, -1 on incorrect speed, -2 on serial write fail.
*/
int QikAlterBraking(int Braking, int Descriptor, int Side); 


/*
  Function changes how agressively each individual motor changes dirrection.
  Needs to be passed: A slew rate (0 < slew < 90: 0 = immediate change),
                      a file descriptor, and a motor)
  Returns: 0 on Success, -1 on incorrect speed, -2 on serial write fail.
*/
int QikAlterSlew(int Slew, int Descriptor, int Side);


/*
  E-Stop Function to lock up all wheels ICE
  Needs to be passed: Both descriptors.
*/
void QikEStop(int Descriptor);

/*
  Closing function to release a connection to a device.
*/
int QikClose(int Descriptor);

#endif // _qik_h
