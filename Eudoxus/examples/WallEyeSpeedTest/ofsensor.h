#ifndef _OFSENSOR_H
#define _OFSENSOR_H


//walleyesensor
/**/
#define ID_VENDOR            0x046D
#define ID_PRODUCT           0xC068
#define NUMBER_OF_INTERFACES 1
/**/

//HP Mouse
/*
#define ID_VENDOR            0x192F
#define ID_PRODUCT           0x0416
#define NUMBER_OF_INTERFACES 0
/**/


class OFlow {
public:
  OFlow();
  ~OFlow();

  //Function which opens a connection to Optical flow sensor
  //*** Specific to logitech mouse board on prototype sensor *** 
  int OpenOF();

  //Function which Closes the openned sensor and data structures
  void CloseOF();

  /*
   * Function which reads the current X Y flow rates.
   * Needs to be passed: X and Y int variables to be filled/modified
   * (values are in radians) 
  */
  void ReadOF(int &ex, int &why);

  //Offered variables to store user accessible flow rates

private:
  struct libusb_device_handle     *OF_lock;
  struct libusb_transfer          *Trans;
  int exx, whyy;
  unsigned char Buff[20];

  static void cbof(struct libusb_transfer *trans);
};

#endif
