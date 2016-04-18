#include "rnr/imu/yost.h"

#include <libusb-1.0/libusb.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/time.h>
#include <sys/ioctl.h>
#include <time.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <termios.h>
#include <string.h>
#include <fcntl.h>
#include <errno.h>
#include <unistd.h>

using namespace std;

//Option left open for constuctor function
Yost::Yost() {
  USB_lock = NULL;
  FD = -1;
}

Yost::~Yost() {
  if(FD >= 0) {
    close(FD);
  }
  FD = -1;
}


//Returns -1 if no YOST sensor on USB
int Yost::checkYost() {
  libusb_init(&tmpcntxt);
  libusb_set_debug(tmpcntxt, 1);
  //Looking for a USB devive with same vendor and product ID as YOST
  USB_lock = libusb_open_device_with_vid_pid(tmpcntxt, YOST_ID_VENDOR,
                                                   YOST_ID_PRODUCTA);
  if(USB_lock == NULL) {
    USB_lock = libusb_open_device_with_vid_pid(tmpcntxt, YOST_ID_VENDOR,
                                                   YOST_ID_PRODUCTB);
    if(USB_lock == NULL) {
      return -1;
    }
  }
  //Freeing up YOST for use if found
  libusb_close(USB_lock);
  USB_lock = NULL;
  libusb_exit(tmpcntxt);
  return YOST_WE_OK;
}

//Returns -1 if unable to get attributes, -2 on failure to set attributes
int Yost::setAttributes(int fileDesc) {
  struct termios tios;
  int error;
  error = tcgetattr(fileDesc, &tios);
  if(error < 0) {
    return -1;
  }
  cfsetispeed(&tios, B115200); 
  cfsetospeed(&tios, B115200);
  tios.c_cflag |= (CLOCAL|CREAD); 
  tios.c_cflag &= ~PARENB; 
  tios.c_cflag &= ~CSIZE;
  tios.c_cflag |= CS8;          
  tios.c_cflag &= ~CRTSCTS;
  tios.c_iflag &= ~(IXON|IXOFF|IXANY);
  //Submitting terminal attributes for communication
  error = tcsetattr(fileDesc, TCSANOW, &tios);
  if(error < 0) {
    return -2;
  }
  return YOST_WE_OK;
}


//Returns: -1 on complete write failure, -2 if no bytes written,
//-3 if number of bytes written is not correct, -4 on read failure
int Yost::writeNRead(uint8_t *from, const char *to) {
  size_t length = strlen(to);
  int err;
  err = write(FD, to, length);
  if(err < 0) {
    return -1;
  }
  if(err == 0) {
    return -2;
  }
  if(err != length) {
    return -3;
  }
  usleep(3000);
  //Mandatory sleep length for a write from the YOST
  err = read(FD,from,3000);
  if(err <= 0) {
    return -4;
  }
  return YOST_WE_OK;
}

//Resets factory defaults on Yost
int Yost::setYostToDefault() {
  const char *out = ":224\n";
  size_t length = strlen(out);
  int err;
  err =write(FD, out, length);
  if(err < 0) {
    return -1;
  }
  return YOST_WE_OK;
}


//Returns -1 if unable to open designated device, 
//-2 if unable to set attributes, -3 on transfer failure, 
//-4 if device is not YOST
int Yost::checkIfYost(const char *dev) { 
  int err;  
  uint8_t buff[40];
  FD = open(dev, O_RDWR | O_NONBLOCK | O_NOCTTY);
  if(FD < 0) {
    return -1;
  }
  err = setAttributes(FD);
  if(err < 0) {
   close(FD);
   return -2;
  } 
  usleep(500);
  //Sending out a version check
  err = writeNRead(buff, ":230\n");
  if(err < 0){
    close(FD);
    return -3;
  }
  //Seeing if first byte recieved is correct
  //(could be more specific, however the likelihood of another product
  //returning the same byte given our version prompt is pretty low)
  if(buff[0] != 0x54) {
    close(FD);
    return -4;
  }
  close(FD);
  return YOST_WE_OK;
}

//Returns: -1 if no YOST on USB, -2 if yost is not at ttyACM0 or ttyACM1
int Yost::OpenYost() {
  int er;
  er = checkYost();
  if( er < 0) {
    return -1;
  }
  //Assuming a potential of multiple ttyACM* devices
  er = checkIfYost("/dev/ttyACM0");
  if(er == 0) {
    FD = open("/dev/ttyACM0", O_RDWR | O_NONBLOCK | O_NOCTTY);
    if(FD < 0) {
      return -1;
    }
    setYostToDefault();
    resetKalman();
    usleep(1000);
    ResetAngles();
    return YOST_WE_OK;
  }
  er = checkIfYost("/dev/ttyACM1");
  if(er == 0) {
    FD = open("/dev/ttyACM1", O_RDWR | O_NONBLOCK | O_NOCTTY);
    if(FD < 0) {
      return -1;
    }
    setYostToDefault();
    resetKalman();
    usleep(1000);
    ResetAngles();
    return YOST_WE_OK;
  }
  return -2;
}


//Quick close function to free the device
void Yost::CloseYost() {
  if(FD >= 0) {
    close(FD);
  }
  FD = -1;
}

//Returns -1 on failure to set 0 point
int Yost::ResetAngles() {
  const char *out = ":96\n";
  size_t length = strlen(out);
  int err;
  err =write(FD, out, length);
  if(err < 0) {
    return -1;
  }
  return YOST_WE_OK;
}

//Returns -1 on failure to disengage the magnetometer
int Yost::MagOff() {
  const char *out = ":109,0\n";
  size_t length = strlen(out);
  int err;
  err =write(FD, out, length);
  if(err < 0) {
    return -1;
  }
  return YOST_WE_OK;
}

//Returns -1 on failure to disengage the magnetometer
int Yost::MagOn() {
  const char *out = ":109,1\n";
  size_t length = strlen(out);
  int err;
  err =write(FD, out, length);
  if(err < 0) {
    return -1;
  }
  return YOST_WE_OK;
}

//returns -1 on transfer error
int Yost::ReadEuler(float &ex, float &why, float &zee) {
  uint8_t buffr[YOST_MAX_BUFF_LEN];
  int err;
  err = writeNRead(buffr, ":1\n");
  if(err < 6) {
    return -1;
  }
  //Converting buffer to three float values (X, Y, Z)
  const char * inString = strtok((char *)buffr, ",");
  if(inString != NULL) {
    ex = atof(inString);
  }
  else {
    return -2;
  }
  inString = strtok(NULL, ",");
  if(inString != NULL) {
    why = atof(inString);
  }
  else {
    return -2;
  }
  inString = strtok(NULL, ",");
  if(inString != NULL) {
    zee = atof(inString);
  }
  else {
    return -2;
  }
  return YOST_WE_OK;
}


//returns -1 on transfer error
int Yost::ReadQuat(float &dubya, float &ex, float &why, float &zee) {
  uint8_t buffr[YOST_MAX_BUFF_LEN];
  int err;
  err = writeNRead(buffr, ":0\n");
  if(err < 0) {
    return -1;
  }
  //Converting buffer to three float values (X, Y, Z)
  const char * inString = strtok((char *)buffr, ",");
  if(inString != NULL) {
    ex = atof(inString);
  }
  else {
    return -2;
  }
  inString = strtok(NULL, ",");
  if(inString != NULL) {
    why = atof(inString);
  }
  else {
    return -2;
  }
  inString = strtok(NULL, ",");
  if(inString != NULL) {
    zee = atof(inString);
  }
  else {
    return -2;
  }
  inString = strtok(NULL, ",");
  if(inString != NULL) {
    dubya = atof(inString);
  }
  else {
    return -2;
  }
  return YOST_WE_OK;
}



//returns -1 on transfer error
int Yost::ReadVectors(float &downx, float &downy, float &downz,
                      float &forx, float &fory, float &forz) {
  uint8_t buffr[YOST_MAX_BUFF_LEN];
  int err;
  err = writeNRead(buffr, ":11\n");
  if(err < 0) {
    return -1;
  }
  //Converting buffer to three float values (X, Y, Z)
  const char * inString = strtok((char *)buffr, ",");
  if(inString != NULL) {
    forx = atof(inString);
  }
  else {
    return -2;
  }
  inString = strtok(NULL, ",");
  if(inString != NULL) {
    fory = atof(inString);
  }
  else {
    return -2;
  }
  inString = strtok(NULL, ",");
  if(inString != NULL) {
    forz = atof(inString);
  }
  else {
    return -2;
  }
  inString = strtok(NULL, ",");
  if(inString != NULL) {
    downx = atof(inString);
  }
  else {
    return -2;
  }
  inString = strtok(NULL, ",");
  if(inString != NULL) {
    downy = atof(inString);
  }
  else {
    return -2;
  }
  inString = strtok(NULL, ",");
  if(inString != NULL) {
    downz = atof(inString);
  }
  else {
    return -2;
  }
  return YOST_WE_OK;
}

//returns -1 on transfer error
int Yost::ReadDecoupleAngles(float &yaw, float &pitch, float &roll) {
  float forx, fory, forz, downx, downy, downz;
  double tmpx=0, tmpy=0, tmpz=0;
  uint8_t buffr[YOST_MAX_BUFF_LEN];
  int err;
  err = writeNRead(buffr, ":11\n");
  if(err < 0) {
    return -1;
  }
  const char * inString = strtok((char *)buffr, ",");
  if(inString != NULL) {
    forx = atof(inString);
  }
  else {
    return -2;
  }
  inString = strtok(NULL, ",");
  if(inString != NULL) {
    fory = atof(inString);
  }
  else {
    return -2;
  }
  inString = strtok(NULL, ",");
  if(inString != NULL) {
    forz = atof(inString);
  }
  else {
    return -2;
  }
  inString = strtok(NULL, ",");
  if(inString != NULL) {
    downx = atof(inString);
  }
  else {
    return -2;
  }
  inString = strtok(NULL, ",");
  if(inString != NULL) {
    downy = atof(inString);
  }
  else {
    return -2;
  }
  inString = strtok(NULL, ",");
  if(inString != NULL) {
    downz = atof(inString);
  }
  else {
    return -2;
  }
  tmpx = (double)forx;
  tmpy = (double)forz;
  normalize(&tmpx,&tmpy,&tmpz);
  yaw = atan2(tmpx,tmpy);
  tmpx = (double)downx;
  tmpy = (-1.00)*(double)downy;
  normalize(&tmpx,&tmpy,&tmpz);
  pitch = atan2(tmpx,tmpy);
  tmpx = (double)downz;
  tmpy = (-1.00)*(double)downy;
  normalize(&tmpx,&tmpy,&tmpz);
  roll = atan2(tmpx,tmpy);
  return YOST_WE_OK;
}


//Returns -1 on transfer error
int Yost::ReadAccel(float &ex, float &why, float &zee) {
  uint8_t buffr[YOST_MAX_BUFF_LEN];
  int err;
  err = writeNRead(buffr, ":66\n");
  if(err < 0) {
    return -1;
  }
  //Converting buffer to three float values (X, Y, Z)
  const char * inString = strtok((char *)buffr, ",");
  if(inString != NULL) {
    ex = atof(inString);
  }
  inString = strtok(NULL, ",");
  if(inString != NULL) {
    why = atof(inString);
  }
  inString = strtok(NULL, ",");
  if(inString != NULL) {
    zee = atof(inString);
  }
  return YOST_WE_OK;
}


//Returns -1 on transfer error
int Yost::ReadConfidence(float &cee) {
  uint8_t buffr[YOST_MAX_BUFF_LEN];
  int err;
  err = writeNRead(buffr, ":38\n");
  if(err < 0) {
    return -1;
  }
  //Converting buffer to float value 
  const char * inString = strtok((char *)buffr, ",");
  if(inString != NULL) {
    cee = atof(inString);
  }
  return YOST_WE_OK;
}


//Returns -1 on transfer error
int Yost::ReadTemp(int &tee) {
  uint8_t buffr[YOST_MAX_BUFF_LEN];
  int err;
  err = writeNRead(buffr, ":37\n");
  if(err < 0) {
    return -1;
  }
  //Converting buffer to float value 
  const char * inString = strtok((char *)buffr, ",");
  if(inString != NULL) {
    tee = atoi(inString);
  }
  return YOST_WE_OK;
}


int Yost::gyroSpeed(int speed) {
  const char *out;
  if(speed == 0){
    out = ":125,0\n";
  }
  if(speed == 1){
    out = ":125,1\n";
  }
  if(speed == 2){
    out = ":125,2\n";
  }
  size_t length = strlen(out);
  int err;
  err =write(FD, out, length);
  if(err < 0) {
    return -1;
  }
  return YOST_WE_OK;
}


int Yost::calibGyro() {
  const char *out = ":165\n";
  size_t length = strlen(out);
  int err;
  err =write(FD, out, length);
  usleep(5000);
  if(err < 0) {
    return -1;
  }
  return YOST_WE_OK;
}

int Yost::resetKalman() {
  const char *out = ":120\n";
  size_t length = strlen(out);
  int err;
  err =write(FD, out, length);
  usleep(5000);
  if(err < 0) {
    return -1;
  }
  return YOST_WE_OK;
}

void Yost::normalize(double *nx, double *ny, double *nz) {
  float mag;
  mag = sqrt(((*nx)*(*nx))+((*ny)*(*ny))+((*nz)*(*nz)));
  if(mag != 0) {
    *nx = (*nx)/mag;
    *ny = (*ny)/mag;
    *nz = (*nz)/mag;
  }
}
