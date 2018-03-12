#include <libsysfs.h>
#include <stdio.h>

#include <time.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <stdlib.h>
#include <stdint.h>
typedef uint8_t u8;
typedef uint16_t u16;
#include "twl4030-madc.h"

#define INPUT_RANGE 10.34f


int main (void)
{
  FILE * fpLog;
  fpLog = fopen ("/home/root/voltage.log", "w");
  if (fpLog == NULL) {
    printf("failed to open log file.\n");
    return 2;
  }

  int d = open("/dev/twl4030-madc", O_RDWR | O_NONBLOCK);
  if (d == -1)
  {
    printf ("could not open device\n");
    return 1;
  }


  /* int i;
  } while (i == 1); // 1 == plugged in */
  // Start logging
  fputs ("Started logging...\n", fpLog);
  
  struct twl4030_madc_user_parms *par;
  par = malloc(sizeof(struct twl4030_madc_user_parms));
  memset(par, 0, sizeof(struct twl4030_madc_user_parms));
  par->channel = 2; // ADCIN2 seems to show the system voltage ToDo: find batt
  int ret;
  
  clock_t prevTime = clock();
  int count = 0;

  while (1) {
    if ( (clock() - prevTime) >= CLOCKS_PER_SEC) {
      prevTime = clock();
      ret = ioctl(d, TWL4030_MADC_IOCX_ADC_RAW_READ, par);
      float result = ((unsigned int)par->result) / 1024.f; // 10 bit ADC -> 1024

      if (ret == 0 && par->status != -1) {
        result *= INPUT_RANGE;
        fprintf (fpLog, "%f\n", result);
        fflush (fpLog);
      }

      if (++count >= 60) {
        count = 0;
        printf("Still working...voltage is %f\n", result);
      }
    }
  }

  fclose (fpLog);
  return 0;
}
  
