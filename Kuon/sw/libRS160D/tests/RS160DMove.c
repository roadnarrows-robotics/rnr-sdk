#include "Kuon/RS160DControl.h"

int error = 0;

struct RS160Ds MyController;

int main() {
  int error;
  error = OpenRS160DSerial("/dev/ttyUSB0","/dev/ttyUSB1",&MyController);
  if(error < 0) {
    exit(1);
  }
  error = SetRS160DToSerial(MyController.m_fdFront);
  if(error < 0) {
    exit(1);
  }
  error = SetRS160DToSerial(MyController.m_fdRear);
  if(error < 0) {
    exit(1);
  }
  error = UpdateRS160DMotorSpeeds( -100, MyController.m_fdFront, 0);
  if(error < 0) {
    exit(1);
  }
  error = UpdateRS160DMotorSpeeds( 100, MyController.m_fdFront, 1);
  if(error < 0) {
    exit(1);
  }
  error = UpdateRS160DMotorSpeeds( -100, MyController.m_fdRear, 0);
  if(error < 0) {
    exit(1);
  }
  error = UpdateRS160DMotorSpeeds( 100, MyController.m_fdRear, 1);
  if(error < 0) {
    exit(1);
  }

  sleep(5);
  error = UpdateRS160DMotorSpeeds( 0, MyController.m_fdFront, 0);
  if(error < 0) {
    exit(1);
  }
  error = UpdateRS160DMotorSpeeds( 0, MyController.m_fdFront, 1);
  if(error < 0) {
    exit(1);
  }
  error = UpdateRS160DMotorSpeeds( 0, MyController.m_fdRear, 0);
  if(error < 0) {
    exit(1);
  }
  error = UpdateRS160DMotorSpeeds( 0, MyController.m_fdRear, 1);
  if(error < 0) {
    exit(1);
  }
  return 0;
}
