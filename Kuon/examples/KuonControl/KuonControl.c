#include "Kuon/RS160DControl.h"
#include "rnr/hid/360Controller.h"
#include "rnr/rnrconfig.h"
#include "rnr/log.h"

#define PERCENT 1

struct Control360 hid360;
int error;

int GetLeftValue(int Why, int Ex) {
  int Return;
  if(Why == 0) {
    Return = (-.6)*Ex;
    Return = Return * PERCENT;
    return Return;
  }
  Return = (.7 * Why) - (.6 * Ex);
  Return = Return * PERCENT;
  return Return;
}

int GetRightValue( int Why, int Ex) {
  int Return;
  if(Why == 0) {
    Return = .6 * Ex;
    Return = Return * PERCENT;
    return Return;
  }
  Return = (.7 * Why) + (.6 * Ex);
  Return = Return * PERCENT;
  return Return;
}

int main() {
  struct RS160Ds MyMotors;
  bool_t coast = false;
  int error;
  LOGDIAG2("Testing");
  error = RS160DOpenConnection("/dev/ttyUSB0",&(MyMotors.m_fdFront));
  if(error < 0) {
    LOGDIAG2("Failed to open front motor controller");
    exit(1);
  }
  error = RS160DOpenConnection("/dev/ttyUSB1",&(MyMotors.m_fdRear));
  if(error < 0) {
    exit(1);
  }
  error = RS160DSetToSerial(MyMotors.m_fdFront);
  if(error < 0) {
    exit(1);
  }
  error = RS160DSetToSerial(MyMotors.m_fdRear);
  if(error < 0) {
    exit(1);
  }
  Init360Controller(&hid360);

  while(hid360.Controller_Connected){
    Update360ControllerValues();
    if(hid360.Center_X) {
      printf("****Exiting****\n");
      hid360.Controller_Connected = 0;
    }
    if((hid360.Right_Trig_Val >= 100) && !coast) {
      RS160DAlterBraking(0,MyMotors.m_fdFront,0);
      RS160DAlterBraking(0,MyMotors.m_fdFront,1);
      RS160DAlterBraking(0,MyMotors.m_fdRear,0);
      RS160DAlterBraking(0,MyMotors.m_fdRear,1);
      RS160DAlterSlew(40,MyMotors.m_fdRear,0);
      RS160DAlterSlew(40,MyMotors.m_fdRear,1);
      RS160DAlterSlew(40,MyMotors.m_fdFront,0);
      RS160DAlterSlew(40,MyMotors.m_fdFront,1);
      coast = true;
    }
    if((hid360.Right_Trig_Val < 100) && coast) {
      RS160DAlterBraking(31,MyMotors.m_fdFront,0);
      RS160DAlterBraking(31,MyMotors.m_fdFront,1);
      RS160DAlterBraking(31,MyMotors.m_fdRear,0);
      RS160DAlterBraking(31,MyMotors.m_fdRear,1);
      RS160DAlterSlew(0,MyMotors.m_fdRear,0);
      RS160DAlterSlew(0,MyMotors.m_fdRear,1);
      RS160DAlterSlew(0,MyMotors.m_fdFront,0);
      RS160DAlterSlew(0,MyMotors.m_fdFront,1);
      coast = false;
    }

    RS160DUpdateMotorSpeeds(GetLeftValue(hid360.Left_Y_Val, 
                           hid360.Left_X_Val), MyMotors.m_fdFront,
                           1);
    RS160DUpdateMotorSpeeds(-1*GetRightValue(hid360.Left_Y_Val,
                           hid360.Left_X_Val), MyMotors.m_fdFront,
                           0);
    RS160DUpdateMotorSpeeds(GetLeftValue(hid360.Left_Y_Val, 
                           hid360.Left_X_Val), MyMotors.m_fdRear,
                           1);
    RS160DUpdateMotorSpeeds(-1*GetRightValue(hid360.Left_Y_Val,
                           hid360.Left_X_Val), MyMotors.m_fdRear,
                           0);
  }
  RS160DEStop(MyMotors.m_fdFront, MyMotors.m_fdRear);
  RS160DClose(MyMotors.m_fdFront);
  RS160DClose(MyMotors.m_fdRear);
  Kill360Controller(&hid360);
  return 0;
}
