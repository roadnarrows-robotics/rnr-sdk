/*
  Title:  Header file for 360 controller handling
  Authors: Rob Shiely & Robin Knight
  Date:   10/12/2011
*/

#ifndef _360READ_H
#define _360READ_H

#include <sys/types.h>
#include <sys/stat.h>
#include <sys/time.h>
#include <sys/ioctl.h>
#include "rnr/rnrconfig.h"
#include <limits.h>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <stdlib.h>
#include <stdarg.h>
#include <time.h>
#include <libusb-1.0/libusb.h>
#include <stdio.h>
#include <string.h>

#define ID_VENDOR            0x045e
#define ID_PRODUCT_WI        0x028e
#define ID_PRODUCT_WL        0x028f
#define ID_PRODUCT_WR        0x0291
#define NUMBER_OF_INTERFACES 3
#define ENDPOINT             0x81
#define LEFT_DEAD_ZONE       50
#define RIGHT_DEAD_ZONE      25

C_DECLS_BEGIN

typedef struct Control360 {
  struct libusb_device_handle     *USB_lockOn;
  struct libusb_device            *dev;
  struct libusb_device_descriptor *desc;
  struct libusb_transfer          *Trans;

  int Pad_Up;
  int Pad_Down;
  int Pad_Left;
  int Pad_Right;
  int Left_Bump;
  int Right_Bump;
  int Start;
  int Back;
  int Left_Stick_Click;
  int Right_Stick_Click;
  int A_Button;
  int B_Button;
  int X_Button;
  int Y_Button;
  int Center_X;
  int Left_X_Val;
  int Left_Y_Val;
  int Right_X_Val;
  int Right_Y_Val;
  int Left_Trig_Val;
  int Right_Trig_Val;
  int Controller_Connected;
  int type;
  int WorC;
} Control360_T;

void debugController(Control360_T c);

/*
  Initialization function for XBox 360 Controller
  defined by Vendor and Product IDs defined above

  Returns: 0 on success, -1 if unable to locate controller, 
           -2 if unable to claim all interfaces,
           and -3 if unable to set up reading from controller.
*/
int Init360Controller(struct Control360 *Controller);

/*
  Function to check the controller and update passed controller structure.

  Returns: 0 on successful update and < 0 on no-update or failure
*/
int Update360ControllerValues();

/*
  Function to To get raw controller output buffer.
  (also updates controller stucture)

  Returns: Length of transferred buffer on success and < 0 on failure.
*/
int GetRaw360(unsigned char *TempBuff, int LeN);

/*
  Function to release all interfaces to the controller,
  close controller, and exit from libusb.
  
  Returns: 0 on success and -1 on failure.
*/
int Kill360Controller(struct Control360 *Controller);
C_DECLS_END
#endif
