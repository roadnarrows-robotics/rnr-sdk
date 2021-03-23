////////////////////////////////////////////////////////////////////////////////
//
// Package:   Kuon
//
// library:   libqik
//
// File:      qikControl.c
//
/*! \file
 *
 * $LastChangedDate: 2013-01-04 13:34:03 -0700 (Fri, 04 Jan 2013) $
 * $Rev: 2590 $
 *
 * \brief  Funtions for serial control of RS160D motor controller
 *
 * \author Rob Shiely     (rob@roadnarrows.com)
 *
 * \par Copyright:
 * (C) 2012.  University of Northern Colorado
 * \n All Rights Reserved
 */
//
//  TODO: get current draw 
//

#include <stdlib.h>
#include <string.h>
#include <dirent.h>

#include "Kuon/qikControl.h"
#include "rnr/log.h"


int QikOpenConnection( int *Descriptor) {
  *Descriptor = SerDevOpen("/dev/ttyUSB1", BAUDRATE, 8, 'N', 1, false, false);

  return *Descriptor;
}

int WriteToSerial(byte_t *ControlString, int Length, int Descriptor) {
  ssize_t err;
  err = SerDevWrite(Descriptor, (byte_t *)ControlString, (size_t)Length, 0);
  if(err < 0) {
    return -1;
  }
  if(err == 0) {
    return -2;
  }
  if(err != Length) {
    return -3;
  }
  return 0;
}

void SerWriteErrCheck( int err){
  if(err == -1) {
    fprintf(stderr, "Complete serial write failure.\n");
  }
  if(err == -2) {
    fprintf(stderr, "Nothing Written.\n");
  }
  if(err == -3) {
    fprintf(stderr, "Incorrect number of bytes written.\n");
  }
}

int QikUpdateMotorSpeeds(int Speed, int Descriptor, int Side) {
  byte_t SOM = 0xAA;
  byte_t DEVID = 0x0A;
  byte_t dir;
  byte_t BuffToMotor[20];
  int err;

  if(Speed > 250) {
    fprintf(stderr, "Speed (%i) too high.\n", 
             Speed);
    return -1;
  }
  if(Speed < -250) {  
    fprintf(stderr, "Speed (%i) too low.\n", 
             Speed);
    return -1;
  }
  // DHP
  // Speed = Speed/2;
  Speed = Speed;
  
  
  int len = 0;
  BuffToMotor[len] = SOM;
  BuffToMotor[1] = DEVID;
  
  if(Speed < 0) {
    Speed=-1*Speed;
    if(Side == 0) {
      dir = 0x0A;
    }
    if(Side == 1) {
      dir = 0x0E;
    }
  }
  else {
    if(Side == 0) {
      dir = 0x08;
    }
    if(Side == 1) {
      dir = 0x0C;
    }
  }
  BuffToMotor[2] = dir;
  BuffToMotor[3] = (byte_t)Speed;
  err = WriteToSerial(BuffToMotor, 4, Descriptor);
  if(err < 0) {
    fprintf(stderr, "\nFailed to update motor speed.\n");
    SerWriteErrCheck(err);
    return -2;
  }
  usleep(1000);
  return 0;
}

int QikAlterBraking(int Braking, int Descriptor, int Side) {
  byte_t BuffToMotor[20];
  int err;
  if(Braking > 31) {
    fprintf(stderr, "Brake rate (%i) too high.\n", 
             Braking);
    return -1;
  }
  if(Braking < 0) {  
    fprintf(stderr, "Brake rate (%i) too low.\n", 
             Braking);
    return -1;
  }
  err = WriteToSerial(BuffToMotor, 2, Descriptor);
  if(err < 0) {
    fprintf(stderr, "\nFailed to update brake rate.\n");
    SerWriteErrCheck(err);
    return -2;
  }
  return 0;
}

int QikAlterSlew(int Slew, int Descriptor, int Side) {
  byte_t BuffToMotor[20];
  int err;
  if(Slew > 90) {
    fprintf(stderr, "Slew rate (%i) too high.\n", 
             Slew);
    return -1;
  }
  if(Slew < 0) {  
    fprintf(stderr, "Slew rate (%i) too low.\n", 
             Slew);
    return -1;
  }
  err = WriteToSerial(BuffToMotor,2, Descriptor);
  if(err < 0) {
    fprintf(stderr, "\nFailed to update slew rate.\n");
    SerWriteErrCheck(err);
    return -2;
  }
  return 0;
}

void QikEStop(int Descriptor){
  byte_t Bullshit[10];
  Bullshit[0] = 0x03;
  WriteToSerial( Bullshit, 2, Descriptor );
}

int QikClose(int Descriptor) {
  int error;
  SerDevFIFOOutputFlush(Descriptor);
  error = SerDevClose(Descriptor);
  if(error == -1) {
    fprintf(stderr, "\nFailed to close device.\n");
    return -1;
  }
  return 0;
}
