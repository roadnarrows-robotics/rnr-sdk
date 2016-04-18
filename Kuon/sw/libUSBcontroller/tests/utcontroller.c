#include "rnr/hid/360Controller.h"

struct Control360 Karen;

int Error;

int main(){
  int wasAController;
  while(1) {
    Init360Controller(&Karen);
    while(Karen.Controller_Connected){
      wasAController = 1;
      usleep(5000);
      Update360ControllerValues();
      usleep(5000);
      fprintf(stderr,"                                                                       \r");
      fprintf(stderr,"                                               Y:%d\r", Karen.Right_Y_Val);
      fprintf(stderr,"                                      X:%d\r", Karen.Right_X_Val);
      if(Karen.Start) {
        fprintf(stderr,"                              Start\r");
      }
      if(Karen.Back) {
        fprintf(stderr,"                         Back\r");
      }
      if(Karen.Pad_Left) {
        fprintf(stderr,"                      <\r");
      }
      if(Karen.Pad_Right) {
        fprintf(stderr,"                      >\r");
      }
      if(Karen.Pad_Up) {
        fprintf(stderr,"                      ^\r");
      }
      if(Karen.Pad_Down) {
        fprintf(stderr,"                      v\r");
      }
      if(Karen.A_Button) {
        fprintf(stderr,"                    A\r");
      }
      if(Karen.B_Button) {
        fprintf(stderr,"                  B\r");
      }
      if(Karen.Y_Button) {
        fprintf(stderr,"                Y\r");
      }
      if(Karen.X_Button) {
        fprintf(stderr,"              X\r");
      }
      fprintf(stderr,"       Y:%d\r", Karen.Left_Y_Val);
      fprintf(stderr,"X:%d\r", Karen.Left_X_Val);
      if(Karen.Center_X) {
        printf("\n****Exiting****\n");
        exit(0);
      }
    }
    if(Karen.type == 3){
      wasAController = 1;
    }
    Karen.type = 0;
    if(wasAController){
      fprintf(stderr, "Controller disconnected.\n");
      Kill360Controller(&Karen);
      wasAController = 0;
    }
    fprintf(stderr, "No controller connected.\n");
    sleep(2);
  }
  return 0;
}
