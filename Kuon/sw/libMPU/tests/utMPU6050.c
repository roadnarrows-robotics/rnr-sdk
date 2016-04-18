#include "rnr/MPU6050.h"

#define AVERAGE 8

i2c_t MPU;

struct timeval  TimeStamp;
uint_t          Dt;

//Variables for raw X,Y,Z accelerometer values
float AcX,AcY,AcZ;

//Variables for raw X,Y,Z Gyroscope values
float GyX,GyY,GyZ;

//Variables for averaged X,Y,Z values respectively
float ExA,WhyA,ZeeA;
float ExG,WhyG,ZeeG;


float Pitch, Roll, Yaw;


int main() {
  initMPU6050(&MPU);
  timer_mark(&TimeStamp);
  int count = 0;
  float norm;
  float dt;
  while(1) {
    BulkUpdate(&MPU,&AcX,&AcY,&AcZ,&GyX,&GyY,&GyZ);
    
    count++; 
    ExA  += AcX;
    WhyA += AcY;
    ZeeA += AcZ;
    ZeeG += GyZ;
    
    if(count >= AVERAGE) {

      //Averaging
      ExA  = ExA/AVERAGE;
      WhyA = WhyA/AVERAGE;
      ZeeA = ZeeA/AVERAGE;
      ZeeG = ZeeG/AVERAGE;
      //Normalizing
      norm = sqrt(ExA*ExA + WhyA*WhyA + ZeeA*ZeeA);

      if(norm < .698) {
        ExA  = ExA/norm;
        WhyA = WhyA/norm;
        ZeeA = (ZeeA/norm);
     
        Pitch = atan(ZeeA/WhyA) + 1.5707;
        Pitch = Pitch * (180/3.1416);
        Roll = atan(ZeeA/ExA) - 1.5707;
        Roll = Roll * (-180/3.1416);
        if (Pitch > 90) {
          Pitch = Pitch - 180;
        }
        if (Roll > 90) {
          Roll = Roll - 180;
        }
      }
      
        Dt = timer_elapsed(&TimeStamp);
        dt = (float)Dt * .000001; 
        if(fabs(ZeeG) > .5062) {
          Yaw += ZeeG * 3.375 * dt;
        }
      
      count = 0;
      ExA=0;
      WhyA=0;
      ZeeA=0;
      ZeeG=0;
      timer_mark(&TimeStamp);
    }
    fprintf(stderr, "                           Yaw: %0.2f,\r", Yaw);
    fprintf(stderr, "             Pitch: %0.2f,\r", Pitch);
    fprintf(stderr, "Roll: %0.2f,\n", Roll);


    usleep(15000);
    count++;
  }
}

