#include "rnr/MPU6050.h"




/*
 *
 * Function to calibrate amount of gain required on each axis
 *
*/
void GyCalibrate(i2c_t *sensor) {
  int cnt = 0;
  float aye, bee, cee, exx, whyy, zeee;
  float tempX=0, tempY=0, tempZ=0;
  while(cnt < CALIB_COUNT) {
    BulkUpdate(sensor, &aye, &bee, &cee, &exx, &whyy, &zeee);
    tempX += exx;
    tempY += whyy;
    tempZ += zeee;
    cnt++;
  }
  GyOX = tempX/CALIB_COUNT;
  GyOY = tempY/CALIB_COUNT;
  GyOZ = tempZ/CALIB_COUNT;
}








/*
 *
 * Function to set the register dictated by the variable 'registr' to the
 * value in the buffr variable.
 *
*/
void Seti2cReg(i2c_t *sensor, uint8_t registr, uint8_t buffr) {
  unsigned char buff[3];
  buff[0]=registr;
  buff[1]=buffr;
  i2c_write(sensor, MPU_ADDR, buff, 2);
  usleep(5000);
}






/*
 *
 * Function to gather the current status of the MPU
 *
*/
void BulkUpdate(i2c_t *sensor,
                float *acx, float *acy, float *acz, 
                float *gyx, float *gyy, float *gyz ) {
  byte_t buff[15];
  int temp;
  const byte_t registr = HIGH_AX;
  i2c_transfer(sensor, MPU_ADDR, &registr, 1, buff, 14);
  usleep(5000);
  temp = (((int16_t)buff[0]) << 8) | buff[1];
  *acx = (float)temp/4300;
  temp = (((int16_t)buff[2]) << 8) | buff[3];
  *acy = (float)temp/4300;
  temp = (((int16_t)buff[4]) << 8) | buff[5];
  *acz = (float)temp/4300;
  temp = (((int16_t)buff[8]) << 8) | buff[9];
  *gyx = ((float)temp/100) - GyOX;
  temp = (((int16_t)buff[10]) << 8) | buff[11];
  *gyy = ((float)temp/100) - GyOY;
  temp = (((int16_t)buff[12]) << 8) | buff[13]; 
  *gyz = ((float)temp/40) - GyOZ;
}




void initMPU6050(i2c_t *sensor) {
  //Buffer to store outgoing register settings
  uint8_t buffr;


  i2c_open(sensor, "/dev/i2c-3");


  //Total device reset
  buffr = 128;
  Seti2cReg(sensor, PWR_MGMT_1, buffr);
  sleep(1);



  //Setting Clock. '1' sets to Xgyro clk, '0' sets to base internal clock
  buffr = 0;
  Seti2cReg(sensor, PWR_MGMT_1, buffr);
  usleep(15000);
  


  /*
   * Resetting FIFO (shouldn't be necessary)
  */
  /*
  Buffer = 7;
  Seti2cReg(USER_CTRL, Buffer);
  usleep(15000);
  Buffer = 1;
  Seti2cReg(USER_CTRL, Buffer);
  usleep(15000);
  */



  /*
   * Setting update rate to 50Hz
   *
   * Update rate = 1Khz/(1 + '19')
  */
  buffr = 19;
  usleep(15000);
  Seti2cReg(sensor, SMPRT_DIV, buffr);
  usleep(15000);
  



  //Setting low pass filter to 184Hz for Accel and 188Hz for Gyro
  buffr = 3;
  Seti2cReg(sensor, CONFIG, buffr);
  usleep(15000);




  /*
   * Setting gyro range to +/- 500deg/sec
   * No self test: '8'
   * with self test: '232'
  */
  buffr = 232;
  Seti2cReg(sensor, GYRO_CONFIG, buffr);
  sleep(1);
  



  /*
   * Setting Accelerometer range to +/- 4g
   *
   * Setting High pass filter to 2.5Hz
   *
   * Not Running self test
  */
  buffr = 10;
  Seti2cReg(sensor, ACCEL_CONFIG, buffr);
  usleep(15000);
 

  GyCalibrate(sensor);
}





/*
 *
 * Function to read and return a single 2s compliment variable from designated
 * High and low registers
 *
*/
float ReadSingleValue(i2c_t *sensor, uint8_t highRegistr, uint8_t lowRegistr) {
  unsigned char high;
  unsigned char low;
  int retVal;
  i2c_transfer(sensor, MPU_ADDR, &highRegistr, 1, &high, 1);
  usleep(5000);
  i2c_transfer(sensor, MPU_ADDR, &lowRegistr, 1, &low, 1);
  retVal = low | (high<<8);
  usleep(5000);
  return (float)retVal;
}




/*
 *
 * Function to get a reference time for a Dt
 *
*/
void timer_mark(struct timeval  *pTvMark) {
  if( gettimeofday(pTvMark, NULL) != OK ) {
    timerclear(pTvMark);
  }
}





/*
 *
 * Function to find Dt between when it is called and when last reference
 * time was set with timer_mark
 *
*/
uint_t timer_elapsed(struct timeval *pTvMark) {
  struct timeval  tvEnd, tvDelta;

  timer_mark(&tvEnd);

  if( !timerisset(pTvMark) || !timerisset(&tvEnd) ) {
    return UINT_MAX;
  }

  tvDelta.tv_sec = tvEnd.tv_sec - pTvMark->tv_sec;
  if( tvEnd.tv_usec < pTvMark->tv_usec ) {
    tvDelta.tv_sec--;
    tvEnd.tv_usec += 1000000;
  }
  tvDelta.tv_usec = tvEnd.tv_usec - pTvMark->tv_usec;

  return (uint_t)(tvDelta.tv_sec * 1000000 + tvDelta.tv_usec);
}







