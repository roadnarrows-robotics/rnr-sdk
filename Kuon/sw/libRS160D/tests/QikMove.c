#include "Kuon/qikControl.h"

int error = 0;

struct Qiks MyController;

int main() {
  int error;
  error = QikOpenConnection(&MyController.m_fdQiks);
  if(error < 0) {
    exit(1);
  }
  error = QikUpdateMotorSpeeds( 0, MyController.m_fdQiks, 0);
  if(error < 0) {
    exit(1);
  }
  usleep(3000);
  error = QikUpdateMotorSpeeds( 0, MyController.m_fdQiks, 1);
  if(error < 0) {
    exit(1);
  }
  sleep(1);
  error = QikUpdateMotorSpeeds( 40, MyController.m_fdQiks, 0);
  if(error < 0) {
    exit(1);
  }
  usleep(3000);
  error = QikUpdateMotorSpeeds( 40, MyController.m_fdQiks, 1);
  if(error < 0) {
    exit(1);
  }
  sleep(1);
  error = QikUpdateMotorSpeeds( 80, MyController.m_fdQiks, 0);
  if(error < 0) {
    exit(1);
  }
  usleep(3000);
  error = QikUpdateMotorSpeeds( 80, MyController.m_fdQiks, 1);
  if(error < 0) {
    exit(1);
  }
  sleep(1);
  error = QikUpdateMotorSpeeds( 120, MyController.m_fdQiks, 0);
  if(error < 0) {
    exit(1);
  }
  usleep(3000);
  error = QikUpdateMotorSpeeds( 120, MyController.m_fdQiks, 1);
  if(error < 0) {
    exit(1);
  }
  sleep(1);
  error = QikUpdateMotorSpeeds( 160, MyController.m_fdQiks, 0);
  if(error < 0) {
    exit(1);
  }
  usleep(3000);
  error = QikUpdateMotorSpeeds( 160, MyController.m_fdQiks, 1);
  if(error < 0) {
    exit(1);
  }
  sleep(1);

  error = QikUpdateMotorSpeeds( 200, MyController.m_fdQiks, 0);
  if(error < 0) {
    exit(1);
  }
  usleep(3000);
  error = QikUpdateMotorSpeeds( -200, MyController.m_fdQiks, 1);
  if(error < 0) {
    exit(1);
  }
  

  sleep(5);
  error = QikUpdateMotorSpeeds( 0, MyController.m_fdQiks, 0);
  if(error < 0) {
    exit(1);
  }
  usleep(3000);
  error = QikUpdateMotorSpeeds( 0, MyController.m_fdQiks, 1);
  if(error < 0) {
    exit(1);
  }
  QikClose(MyController.m_fdQiks);
  return 0;
}
