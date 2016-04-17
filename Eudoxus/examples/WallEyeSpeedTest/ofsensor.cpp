#include "ofsensor.h"
#include <libusb-1.0/libusb.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/time.h>
#include <sys/ioctl.h>
#include <time.h>
#include <stdio.h>
#include <stdlib.h>
#include <termios.h>
#include <string.h>
#include <fcntl.h>
#include <errno.h>
#include <unistd.h>

using namespace std;


//Debugging function to reverse engineer USB message
void printBits(unsigned char buff[]){
  int count, mask;
  for(count=0, mask=0x80;count < 8; count++, mask >>= 1) {
    if(count==4) {
      fprintf(stderr," ");
    }
    if(mask & buff[0]) {
      fprintf(stderr,"1");
    }
    else {
      fprintf(stderr,"0");
    }
  }
  fprintf(stderr,"  ");
  for(count=0, mask=0x80;count < 8; count++, mask >>= 1) {
    if(count==4) {
      fprintf(stderr," ");
    }
    if(mask & buff[1]) {
      fprintf(stderr,"1");
    }
    else {
      fprintf(stderr,"0");
    }
  }
  fprintf(stderr,"  ");
  for(count=0, mask=0x80;count < 8; count++, mask >>= 1) {
    if(count==4) {
      fprintf(stderr," ");
    }
    if(mask & buff[2]) {
      fprintf(stderr,"1");
    }
    else {
      fprintf(stderr,"0");
    }
  }
  fprintf(stderr,"  ");
  for(count=0, mask=0x80;count < 8; count++, mask >>= 1) {
    if(count==4) {
      fprintf(stderr," ");
    }
    if(mask & buff[3]) {
      fprintf(stderr,"1");
    }
    else {
      fprintf(stderr,"0");
    }
  }
  fprintf(stderr,"  ");
  for(count=0, mask=0x80;count < 8; count++, mask >>= 1) {
    if(count==4) {
      fprintf(stderr," ");
    }
    if(mask & buff[4]) {
      fprintf(stderr,"1");
    }
    else {
      fprintf(stderr,"0");
    }
  }
  fprintf(stderr,"  ");
  for(count=0, mask=0x80;count < 8; count++, mask >>= 1) {
    if(count==4) {
      fprintf(stderr," ");
    }
    if(mask & buff[5]) {
      fprintf(stderr,"1");
    }
    else {
      fprintf(stderr,"0");
    }
  }
  fprintf(stderr,"\n");
}

OFlow::OFlow() {

  OF_lock = NULL;
  Trans = NULL;

};



OFlow::~OFlow() {
  
  CloseOF();

};


void OFlow::cbof(struct libusb_transfer *trans) {
  OFlow *oflow = (OFlow*) trans->user_data;
  oflow->exx=0;
  oflow->whyy=0;

  switch(trans->status) {
    case LIBUSB_TRANSFER_ERROR:
      break;
    case LIBUSB_TRANSFER_NO_DEVICE:
      break;
    case LIBUSB_TRANSFER_TIMED_OUT:
      break;
    case LIBUSB_TRANSFER_CANCELLED:
      break;
    case LIBUSB_TRANSFER_STALL:
      break;
    case LIBUSB_TRANSFER_OVERFLOW:
      break;
    default:
      //printBits(oflow->Buff);
      
      

      /* walleye sensor*/ 
      oflow->exx = ((oflow->Buff[3]) << 8) | oflow->Buff[2];
      oflow->whyy = ((oflow->Buff[5]) << 8) | oflow->Buff[4];
      if(oflow->exx & 0x8000) {
        oflow->exx = oflow->exx - 0xffff;
      }
      if(oflow->whyy & 0x8000) {
        oflow->whyy = oflow->whyy - 0xffff;
      }
      /**/



      /*hp mouse 
      oflow->exx = ((0x0f & oflow->Buff[2]) << 8) | oflow->Buff[1];
      if(oflow->exx & 0x0800) {
        oflow->exx = oflow->exx - 0x0fff;
      }
      oflow->whyy = ((0xf0 & oflow->Buff[2]) >> 4) | 
                    ((0xff & oflow->Buff[3]) << 4);
      if(oflow->whyy & 0x0800) {
        oflow->whyy = oflow->whyy - 0x0fff;
      }
      /**/


      break; 
  }
  libusb_submit_transfer(trans); 
}




//Returns: -1 on failure to open sensor, -2 if a kernel driver would not
//let go,-3 if unable to claim an interface, -4 if unable to allocate a
//transfer, -5 if unable to submit a transfer
int OFlow::OpenOF() {
  int err;
  libusb_init(NULL);
  libusb_set_debug(NULL, 3);
  OF_lock = libusb_open_device_with_vid_pid(NULL,ID_VENDOR,ID_PRODUCT);
  if(OF_lock == NULL) {
    return -1;
  }
  int x=0;
  for( x=0; x <= NUMBER_OF_INTERFACES; x++ ) {
    err = libusb_kernel_driver_active(OF_lock, x);
    if(err > 0) {
      err = libusb_detach_kernel_driver(OF_lock, x);
      if(err < 0) {
        return -2;
      }
    }
  }
  for( x=0; x <= NUMBER_OF_INTERFACES; x++) {
    err = libusb_claim_interface(OF_lock, x);
    if(err < 0) {
      return -3;
    }
  }
  Trans = libusb_alloc_transfer(0);
  if(Trans == NULL) {
    return -4;
  }
  /* walleye sensor */ 
  libusb_fill_interrupt_transfer(Trans, OF_lock,
                                 0x81, Buff, 8, cbof, this, 100);
  /**/
  /* HP Mouse *
  libusb_fill_interrupt_transfer(Trans, OF_lock,
                                 0x81, Buff, 6, cbof, this, 100);
  /**/
  err = libusb_submit_transfer(Trans);
  if(err < 0) {
    return -5;
  }
  //flushing anomalies out of the buffer
  for(x=0;x<=3;x++) {
    libusb_handle_events(NULL);
  }
}



void OFlow::CloseOF() {
  
  if(OF_lock != NULL) {
    libusb_close(OF_lock);
    OF_lock = NULL;
  }
  if(Trans != NULL) {
    libusb_free_transfer(Trans);
    Trans = NULL;
    libusb_exit(NULL);
  }
  
}



void OFlow::ReadOF(int &ex, int &why) {
  if((OF_lock == NULL) || (Trans == NULL)) {
    return;
  }
  libusb_handle_events(NULL);
  ex = exx;
  why = whyy;
}




