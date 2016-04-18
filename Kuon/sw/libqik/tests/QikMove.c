#include "Kuon/qikControl.h"

int error = 0;

//struct Qiks MyController;
int fd;

int main() {
  int error;
  error = QikOpenConnection(&fd);
  if(error < 0) {
    fprintf(stderr, "dhp - failed to open connection.\n");
    exit(1);
  }
  error = QikUpdateMotorSpeeds( 0, fd, 0);
  if(error < 0) {
    exit(1);
  }
  usleep(3000);
  error = QikUpdateMotorSpeeds( 0, fd, 1);
  if(error < 0) {
    exit(1);
  }
  sleep(1);
  error = QikUpdateMotorSpeeds( 40, fd, 0);
  if(error < 0) {
    exit(1);
  }
  usleep(3000);
  error = QikUpdateMotorSpeeds( 40, fd, 1);
  if(error < 0) {
    exit(1);
  }
  sleep(1);
  error = QikUpdateMotorSpeeds( 80, fd, 0);
  if(error < 0) {
    exit(1);
  }
  usleep(3000);
  error = QikUpdateMotorSpeeds( 80, fd, 1);
  if(error < 0) {
    exit(1);
  }
  sleep(1);
  error = QikUpdateMotorSpeeds( 120, fd, 0);
  if(error < 0) {
    exit(1);
  }
  usleep(3000);
  error = QikUpdateMotorSpeeds( 120, fd, 1);
  if(error < 0) {
    exit(1);
  }
  sleep(1);
  error = QikUpdateMotorSpeeds( 160, fd, 0);
  if(error < 0) {
    exit(1);
  }
  usleep(3000);
  error = QikUpdateMotorSpeeds( 160, fd, 1);
  if(error < 0) {
    exit(1);
  }
  sleep(1);

  error = QikUpdateMotorSpeeds( 200, fd, 0);
  if(error < 0) {
    exit(1);
  }
  usleep(3000);
  error = QikUpdateMotorSpeeds( -200, fd, 1);
  if(error < 0) {
    exit(1);
  }
  

  sleep(5);
  error = QikUpdateMotorSpeeds( 0, fd, 0);
  if(error < 0) {
    exit(1);
  }
  usleep(3000);
  error = QikUpdateMotorSpeeds( 0, fd, 1);
  if(error < 0) {
    exit(1);
  }
  QikClose(fd);
  return 0;
}
