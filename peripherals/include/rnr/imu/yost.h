#ifndef _YOST_H
#define _YOST_H

#include <libusb-1.0/libusb.h>
using namespace std;

#define YOST_ID_VENDOR            0x2476
#define YOST_ID_PRODUCTA          0x1010
#define YOST_ID_PRODUCTB          0x1040
#define YOST_NUMBER_OF_INTERFACES 3
#define YOST_READ_WRITE_T         3000
#define YOST_MAX_BUFF_LEN         75
#define YOST_WE_OK                0
class Yost {
public:
  Yost();
  ~Yost();
  /*
   * Function which detects if a YOST is attached to the system. 
   * If one is found it searches for and binds to it.
  */
  int OpenYost();

  /*
   * Function which frees a connection to a previously bound YOST
  */
  void CloseYost();

  /*
   * Function which sets the current attitude & heading as the reference
  */
  int ResetAngles();
  
  /*
   * Function which disengages the magnetometer from the orientation assesment. 
  */
  int MagOff();

  /*
   * Function which disengages the magnetometer from the orientation assesment. 
  */
  int MagOn();

  /*
   * Function which reads the tared euler angles for X Y and Z.
   * Needs to be passed: X, Y, and Z, float variables to be filled/modified
   * (values are in radians) 
  */
  int ReadEuler(float &ex, float &why, float &zee);

  /*
   * Function which reads the tared quaternion W,X,Y,Z.
   * Needs to be passed: W, X, Y, and Z, float variables to be filled/modified
   * (values are in radians) 
  */
  int ReadQuat(float &dubya, float &ex, float &why, float &zee);


  /*
   * Function which reads in two unit vectors, one which indicates down with respect
   * to the sensor, and one which indicates forward with respect to the sensor.
  */
  int ReadVectors(float &downx, float &downy, float &downz,
                      float &forx, float &fory, float &forz);


  int ReadDecoupleAngles(float &yaw, float &pitch, float &roll);

  /*
   * Function which reads the raw Accelerometer values.
   * Needs to be passed: X, Y, and Z, float variables to be filled/modified 
   * (values are in Gs) 
  */
  int ReadAccel(float &ex, float &why, float &zee);

  /*
   * Function which reads the confidence value as generated by the Kalman.
   * Needs to be passed: a confidence variable. Float to be filled/modified
   * (values from 0->1 [0% to 100%]) 
  */
  int ReadConfidence(float &cee);

  /*
   * Function which reads the temperature value.
   * Needs to be passed: a temperature variable. Float to be filled/modified
   * (value is in Fahrenheit) 
  */
  int ReadTemp(int &tee);


private:
  struct libusb_device_handle     *USB_lock;
  struct libusb_context           *tmpcntxt;
  int FD;

  int checkYost();
  int setAttributes(int fileDesc);
  int writeNRead(uint8_t *from, const char *to);
  int checkIfYost(const char *dev);
  //Function to set the expected speed range of gyro
  //0 = 250 DPS, 1= 500 DPS, 2 = 2000 DPS 
  int gyroSpeed(int speed);
  int setYostToDefault();
  int calibGyro();
  int resetKalman();
  void normalize(double *nx, double *ny, double *nz);
};
#endif
