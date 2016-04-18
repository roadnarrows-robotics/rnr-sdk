#ifndef _MPU6050_H
#define _MPU6050_H

#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <dirent.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/time.h>
#include <sys/ioctl.h>
#include <termios.h>
#include <unistd.h>
#include <stdarg.h>
#include <time.h>
#include <math.h>
#include <stdio.h>
#include "rnr/i2c.h"
#include <limits.h>


#define MPU_ADDR      0x69
#define SMPRT_DIV     0x19
#define CONFIG        0x1A
#define GYRO_CONFIG   0x1B
#define ACCEL_CONFIG  0x1C
#define FIFO_EN       0x23
#define USER_CTRL     0x6A
#define PWR_MGMT_1    0x6B
#define PWR_MGMT_2    0x6C
#define I2C_MST_CTRL  0x24
#define HIGH_AX       0x3B
#define LOW_AX        0x3C
#define HIGH_AY       0x3D
#define LOW_AY        0x3E
#define HIGH_AZ       0x3F
#define LOW_AZ        0x40
#define HIGH_GX       0x43
#define LOW_GX        0x44
#define HIGH_GY       0x45
#define LOW_GY        0x46
#define HIGH_GZ       0x47
#define LOW_GZ        0x48
#define CALIB_COUNT   100


float GyOX = 0.0;
float GyOY = 0.0;
float GyOZ = 0.0;

void initMPU6050(i2c_t *sensor);

void timer_mark(struct timeval  *pTvMark);

uint_t timer_elapsed(struct timeval *pTvMark);

void Seti2cReg(i2c_t *sensor, uint8_t registr, uint8_t buffr);

float ReadSingleValue(i2c_t *sensor, uint8_t highRegistr, uint8_t lowRegistr);

void BulkUpdate(i2c_t *sensor,
                float *acx, float *acy, float *acz, 
                float *gyx, float *gyy, float *gyz );
#endif
