////////////////////////////////////////////////////////////////////////////////
//
// Package:   Kuon
//
// Library:   libUSBController
//
// File:      360Controller.c
//
/*! \file
 *
 * $LastChangedDate: 2010-08-20 11:36:38 -0600 (Fri, 20 Aug 2010) $
 * $Rev: 568 $
 *
 * \brief X-Box 360 controller library.
 *
 * \author Robin Knight   (robin.knight@roadnarrows.com)
 * \author Rob Shiely     (rob@roadnarrows.com)
 * \author Daniel Packard (daniel@roadnarrows.com)
 *
 * \par Copyright:
 * (C) 2009-2010.  RoadNarrows LLC.
 * (http://www.roadnarrows.com)
 * \n All Rights Reserved
 */
// Permission is hereby granted, without written agreement and without
// license or royalty fees, to use, copy, modify, and distribute this
// software and its documentation for any purpose, provided that
// (1) The above copyright notice and the following two paragraphs
// appear in all copies of the source code and (2) redistributions
// including binaries reproduces these notices in the supporting
// documentation.   Substantial modifications to this software may be
// copyrighted by their authors and need not follow the licensing terms
// described here, provided that the new terms are clearly indicated in
// all files where they apply.
//
// IN NO EVENT SHALL THE AUTHOR, ROADNARROWS LLC, OR ANY MEMBERS/EMPLOYEES
// OF ROADNARROW LLC OR DISTRIBUTORS OF THIS SOFTWARE BE LIABLE TO ANY
// PARTY FOR DIRECT, INDIRECT, SPECIAL, INCIDENTAL, OR CONSEQUENTIAL
// DAMAGES ARISING OUT OF THE USE OF THIS SOFTWARE AND ITS DOCUMENTATION,
// EVEN IF THE AUTHORS OR ANY OF THE ABOVE PARTIES HAVE BEEN ADVISED OF
// THE POSSIBILITY OF SUCH DAMAGE.
//
// THE AUTHOR AND ROADNARROWS LLC SPECIFICALLY DISCLAIM ANY WARRANTIES,
// INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND
// FITNESS FOR A PARTICULAR PURPOSE. THE SOFTWARE PROVIDED HEREUNDER IS ON AN
// "AS IS" BASIS, AND THE AUTHORS AND DISTRIBUTORS HAVE NO OBLIGATION TO
// PROVIDE MAINTENANCE, SUPPORT, UPDATES, ENHANCEMENTS, OR MODIFICATIONS.
//
////////////////////////////////////////////////////////////////////////////////

#include <sys/types.h>
#include <sys/stat.h>
#include <sys/time.h>

#include "rnr/hid/360Controller.h"
#include "rnr/log.h"

//Error Code
int err = 0;

//Low level buffer for controler pipe.
static unsigned char LowBuff[200] = {'x',};

//Function opens controller
int Find360Controller(struct Control360 *Controller) {
  libusb_init(NULL);
  libusb_set_debug(NULL, 3);
  LOGDIAG3("Looking for Wired 360 Controller.\n");
  Controller->USB_lockOn = libusb_open_device_with_vid_pid(NULL,ID_VENDOR,
                                                         ID_PRODUCT_WI);
  if(Controller->USB_lockOn == NULL) {
    LOGDIAG3("Looking for Tethered Wireless 360 controller.\n");
    Controller->USB_lockOn = libusb_open_device_with_vid_pid(NULL,
                                         ID_VENDOR,ID_PRODUCT_WL);
    if(Controller->USB_lockOn == NULL) {
      LOGDIAG3("Looking for Wireless 360 controller.\n");
      Controller->USB_lockOn = libusb_open_device_with_vid_pid(NULL,ID_VENDOR,
                                                               ID_PRODUCT_WR);
      if(Controller->USB_lockOn == NULL){
        LOGDIAG3("Could not find controller.\n");
        return -1;
      }
      LOGDIAG3("Successfully opened wireless 360 controller reciever. \n");
      Controller->type = 3;
      return 0;
    }
    LOGDIAG3("successfully opened tethered wireless 360 controller.\n");
    Controller->type = 2;
    return 0;
  }
  LOGDIAG3("Successfully opened wired 360 controller.\n");
  Controller->type = 1;
  return 0;
}

//Function claims the USB interfaces to the controller.
int ClaimInterface(struct Control360 *Controller) {
  int x=0;
  for( x=0; x <= NUMBER_OF_INTERFACES; x++ ) {
    err = libusb_kernel_driver_active(Controller->USB_lockOn, x);
    if(err > 0) {
      LOGDIAG3("Kernel driver active on interface %i.\n", x);
      err = libusb_detach_kernel_driver(Controller->USB_lockOn, x);
      if(err < 0) {
        LOGDIAG3("Failed to remove kernel driver from %i.\n", x);
        return -1;
      }
      LOGDIAG3("***Removed***\n");
    }
  }
  if((Controller->type == 1)||(Controller->type == 2)){
    err = libusb_set_configuration(Controller->USB_lockOn, 1);
    if(err < 0) {
      LOGDIAG3("Failed to set 360 controller configuration.\n");
      return -1;
    }
  }
  for( x=0; x <= NUMBER_OF_INTERFACES; x++) {
    err = libusb_claim_interface(Controller->USB_lockOn, x);
    if(err < 0) {
      LOGDIAG3("Failed to claim controller interface %i.\n", x);
      return -1;
    }
  }
  return 0;
}

//Function parses and interprets the incoming buffer from the controller. 
void Interpret360(struct Control360 *Controller) {
  int checkOn;
  if((Controller->type == 1) || (Controller->type == 2)){
    int ModVal;
    Controller->WorC = 1;
    Controller->Pad_Up = ConvertToBit(LowBuff[2] & 0x01);
    Controller->Pad_Down = ConvertToBit(LowBuff[2] & 0x02);
    Controller->Pad_Left = ConvertToBit(LowBuff[2] & 0x04);
    Controller->Pad_Right = ConvertToBit(LowBuff[2] & 0x08);
    Controller->Left_Bump = ConvertToBit(LowBuff[3] & 0x01);
    Controller->Right_Bump = ConvertToBit(LowBuff[3] & 0x02);
    Controller->Start = ConvertToBit(LowBuff[2] & 0x10);
    Controller->Back = ConvertToBit(LowBuff[2] & 0x20);
    Controller->Left_Stick_Click = ConvertToBit(LowBuff[2] & 0x40);
    Controller->Right_Stick_Click = ConvertToBit(LowBuff[2] & 0x80);
    Controller->A_Button = ConvertToBit(LowBuff[3] & 0x10);
    Controller->B_Button = ConvertToBit(LowBuff[3] & 0x20);
    Controller->X_Button = ConvertToBit(LowBuff[3] & 0x40);
    Controller->Y_Button = ConvertToBit(LowBuff[3] & 0x80);
    Controller->Center_X = ConvertToBit(LowBuff[3] & 0x04);
    Controller->Left_X_Val = ((ConvertToInt(LowBuff[6], LowBuff[7]) - 32768)/132);
    Controller->Left_Y_Val = ((ConvertToInt(LowBuff[8], LowBuff[9]) - 32768)/132);
    ModVal = (ConvertToInt(LowBuff[10], LowBuff[11]));
    ModVal = ModVal - 32000;
    Controller->Right_X_Val = ModVal/256;
    ModVal = (ConvertToInt(LowBuff[12], LowBuff[13]));
    ModVal = ModVal - 32000;
    Controller->Right_Y_Val = ModVal/256;
    ModVal = (LowBuff[4]);
    Controller->Left_Trig_Val = ModVal;
    ModVal = (LowBuff[5]);
    Controller->Right_Trig_Val = ModVal;
    if(abs(Controller->Left_X_Val) < LEFT_DEAD_ZONE) {
      Controller->Left_X_Val = 0;
    }
    if(abs(Controller->Left_Y_Val) < LEFT_DEAD_ZONE) {
      Controller->Left_Y_Val = 0;
    }
    if(abs(Controller->Right_X_Val) < RIGHT_DEAD_ZONE) {
      Controller->Right_X_Val = 0;
    }
    if(abs(Controller->Right_Y_Val) < RIGHT_DEAD_ZONE) {
      Controller->Right_Y_Val = 0;
    }
    if(abs(Controller->Left_Trig_Val) == 127) {
      Controller->Left_Trig_Val = 127;
    }
    if(abs(Controller->Right_Trig_Val) == 127) {
      Controller->Right_Trig_Val = 127;
    }
    if(Controller->Left_Trig_Val < 0) {
      Controller->Left_Trig_Val = 127 + (128 - abs(Controller->Left_Trig_Val));
    }
    if(Controller->Right_Trig_Val < 0) {
      Controller->Right_Trig_Val = 127 + (128 - abs(Controller->Right_Trig_Val));
    }
  }
  else if(Controller->type == 3) {
    if((ConvertToBit(LowBuff[0] & 0x08))){
      fprintf(stderr, "couldn't connect\n");
      Controller->Controller_Connected = 0;
    }
    else if(!(LowBuff[1])){
      checkOn = LowBuff[3];
      if(!(checkOn > 0)) {
      fprintf(stderr, "couldn't connect\n");
        Controller->Controller_Connected =0;
      }
    }
    else if (LowBuff[1] || LowBuff[2]) {
      int ModVal;
      ModVal = (LowBuff[18]);
      if(ModVal){
        Controller->WorC = 2;
      fprintf(stderr, "connected\n");
        Controller->Controller_Connected = 1;
        Controller->Pad_Up = ConvertToBit(LowBuff[6] & 0x01);
        Controller->Pad_Down = ConvertToBit(LowBuff[6] & 0x02);
        Controller->Pad_Left = ConvertToBit(LowBuff[6] & 0x04);
        Controller->Pad_Right = ConvertToBit(LowBuff[6] & 0x08);
        Controller->Start = ConvertToBit(LowBuff[6] & 0x10);
        Controller->Back = ConvertToBit(LowBuff[6] & 0x20);
        Controller->A_Button = ConvertToBit(LowBuff[7] & 0x10);
        Controller->B_Button = ConvertToBit(LowBuff[7] & 0x20);
        Controller->X_Button = ConvertToBit(LowBuff[7] & 0x40);
        Controller->Y_Button = ConvertToBit(LowBuff[7] & 0x80);
        Controller->Center_X = ConvertToBit(LowBuff[7] & 0x04);
        Controller->Left_X_Val = ((ConvertToInt(LowBuff[10], LowBuff[11]) - 32768)/132);
        if(abs(Controller->Left_X_Val) <= 11) {
          Controller->Left_X_Val = 0;
        }
        ModVal = (LowBuff[18]);
        if(ModVal <= 127){
          ModVal = (-1) * ModVal;
        }
        else {
          ModVal = 255 - ModVal;
        }
        Controller->Right_Trig_Val = ModVal; 
        ModVal = (LowBuff[8]);
        Controller->Left_Trig_Val = ModVal;
        ModVal = (LowBuff[9]);
        Controller->Left_Y_Val = ModVal;
        if(abs(Controller->Left_Trig_Val) == 127) {
          Controller->Left_Trig_Val = 127;
        }
        if(abs(Controller->Left_Y_Val) == 127) {
          Controller->Left_Y_Val = 127;
        }
        if(Controller->Left_Trig_Val < 0) {
          Controller->Left_Trig_Val = 127 + (128 - abs(Controller->Left_Trig_Val));
        }
        if(Controller->Left_Y_Val < 0) {
          Controller->Left_Y_Val = 127 + (128 - abs(Controller->Right_Trig_Val));
        }
      }
      else {
        Controller->WorC = 1;
      fprintf(stderr, "connected\n");
        Controller->Controller_Connected = 1;
        Controller->Pad_Up = ConvertToBit(LowBuff[6] & 0x01);
        Controller->Pad_Down = ConvertToBit(LowBuff[6] & 0x02);
        Controller->Pad_Left = ConvertToBit(LowBuff[6] & 0x04);
        Controller->Pad_Right = ConvertToBit(LowBuff[6] & 0x08);
        Controller->Left_Bump = ConvertToBit(LowBuff[7] & 0x01);
        Controller->Right_Bump = ConvertToBit(LowBuff[7] & 0x02);
        Controller->Start = ConvertToBit(LowBuff[6] & 0x10);
        Controller->Back = ConvertToBit(LowBuff[6] & 0x20);
        Controller->Left_Stick_Click = ConvertToBit(LowBuff[6] & 0x40);
        Controller->Right_Stick_Click = ConvertToBit(LowBuff[6] & 0x80);
        Controller->A_Button = ConvertToBit(LowBuff[7] & 0x10);
        Controller->B_Button = ConvertToBit(LowBuff[7] & 0x20);
        Controller->X_Button = ConvertToBit(LowBuff[7] & 0x40);
        Controller->Y_Button = ConvertToBit(LowBuff[7] & 0x80);
        Controller->Center_X = ConvertToBit(LowBuff[7] & 0x04);
        Controller->Left_X_Val = ((ConvertToInt(LowBuff[10], LowBuff[11]) - 32768)/132);
        Controller->Left_Y_Val = ((ConvertToInt(LowBuff[12], LowBuff[13]) - 32768)/132);
        ModVal = (ConvertToInt(LowBuff[14], LowBuff[15]));
        Controller->Right_X_Val = ModVal/2;
        ModVal = (ConvertToInt(LowBuff[16], LowBuff[17]));
        Controller->Right_Y_Val = ModVal/2;
        ModVal = (LowBuff[8]);
        Controller->Left_Trig_Val = ModVal;
        ModVal = (LowBuff[9]);
        Controller->Right_Trig_Val = ModVal;
        if(abs(Controller->Left_X_Val) < LEFT_DEAD_ZONE) {
          Controller->Left_X_Val = 0;
        }
        if(abs(Controller->Left_Y_Val) < LEFT_DEAD_ZONE) {
          Controller->Left_Y_Val = 0;
        }
        if(abs(Controller->Right_X_Val) < RIGHT_DEAD_ZONE) {
          Controller->Right_X_Val = 0;
        }
        if(abs(Controller->Right_Y_Val) < RIGHT_DEAD_ZONE) {
          Controller->Right_Y_Val = 0;
        }
        if(abs(Controller->Left_Trig_Val) == 127) {
          Controller->Left_Trig_Val = 127;
        }
        if(abs(Controller->Right_Trig_Val) == 127) {
          Controller->Right_Trig_Val = 127;
        }
        if(Controller->Left_Trig_Val < 0) {
          Controller->Left_Trig_Val = 127 + (128 - abs(Controller->Left_Trig_Val));
        }
        if(Controller->Right_Trig_Val < 0) {
          Controller->Right_Trig_Val = 127 + (128 - abs(Controller->Right_Trig_Val));
        }
      }
    }
  }
}


//CallBack function for the interrupt transfer.
void CB360(struct libusb_transfer *trans) {
  struct Control360 *Controller;
  int i=0;
  int Steer;
  int WhatAmI;
  Controller = (struct Control360 *) (trans->user_data);
  switch(trans->status) {
    case LIBUSB_TRANSFER_COMPLETED:
      /*if(Controller->type == 3){
        for(i=0; i <=20; i++) {
          LOGDIAG3("%02x ",LowBuff[i]);
        }
        LOGDIAG3("\n");
      }*/  
      Interpret360(Controller);
      break;
    case LIBUSB_TRANSFER_ERROR:
      LOGDIAG3("Transfer error.\n");
      break;
    case LIBUSB_TRANSFER_NO_DEVICE:
      LOGDIAG3("CHAIN YANKED!!!\n");
      Controller->Controller_Connected = 0;
      break;
    case LIBUSB_TRANSFER_TIMED_OUT:
      break;
    case LIBUSB_TRANSFER_CANCELLED:
      LOGDIAG3("Tranferer cancelled\n");
      break;
    case LIBUSB_TRANSFER_STALL:
      LOGDIAG3("Transfer stall \n");
      break;
    case LIBUSB_TRANSFER_OVERFLOW:
      LOGDIAG3("Overflow. Amount transferred: %d\n", trans->actual_length);
      break;
    default:
      LOGDIAG3("Unknown error, status: %d\n", trans->status);
      break; 
  }
  libusb_submit_transfer(trans); 
}

//Function sets up an interrupt read from 0x81 endpoint.
int USBSetupReader(struct Control360 *Controller) {
  Controller->Trans = libusb_alloc_transfer(0);
  if(Controller->Trans == NULL) {
    LOGDIAG3("Transfer allocation failed.\n");
    return -1;
  }
  libusb_fill_interrupt_transfer(Controller->Trans, Controller->USB_lockOn,
                                 ENDPOINT, LowBuff, 32, CB360, Controller, 100);
  err = libusb_submit_transfer(Controller->Trans);
  if(err < 0) {
    LOGDIAG3("Failed to submit USB transfer. Error: %d\n", err);
    return -1;
  }

//DHP
  return 0; 
}

//Function converts byte segments into boolean bits.
int ConvertToBit(int NonBit) {
  if(NonBit != 0) {
    return 1;
  }
  return 0;
}

//Function converts a low/high value into an integer.
int ConvertToInt(int one, int two) {
  uint16_t resultant = 32768;
  int final;
  resultant += ((two << 8)| (one) );
  final = resultant;
  return final;
}



//Function executing all necessary functions starting USB communications.
int Init360Controller(struct Control360 *Controller) {
  err = Find360Controller(Controller);
  if(err < 0) {
    return -1;
  }
  err = ClaimInterface(Controller);
  if(err < 0) {
    return -2;
  }
  err = USBSetupReader(Controller);
  if(err < 0) {
    return -3;
  }
  Update360ControllerValues();
  if(Controller->type == 3){
    usleep(500);
    Update360ControllerValues();
  }
  else {
      fprintf(stderr, "connected\n");
    Controller->Controller_Connected = 1;
  }
  return 0;
}

//Function updates controller structure it is passed and fills passed buffer.
int GetRaw360(unsigned char *TempBuff, 
              int LeN) {
  int y;
  int transferred;
  for(y=0; y<=3; y++) {
    err = libusb_handle_events(NULL);
    if(err == 0) {
      memcpy(TempBuff, LowBuff, LeN);
      break;
    }
  }
  transferred = strlen(LowBuff);
  if(err < 0) {
    return err;
  }
  else {
    if(transferred < LeN) {
      return transferred;
    }
    return LeN;
  }
} 

//Function updates current controller structure values.
int Update360ControllerValues() {
  int y;
  for(y=0; y<=3; y++) {
    err = libusb_handle_events(NULL);
    if(err == 0) {
      break;
    }
  }
  return err;
}

//Function ends and releases USB connection
int Kill360Controller(struct Control360 *Controller) {
  int x=0;
  for(x=0; x<=NUMBER_OF_INTERFACES; x++) {
    err = libusb_release_interface(Controller->USB_lockOn, x);
    if(err < 0) {
      LOGDIAG3("Failed to release interface %i.\n", x);
      libusb_close(Controller->USB_lockOn);
      libusb_exit(NULL);
      return -1;
    }
  }
  libusb_close(Controller->USB_lockOn);
  libusb_exit(NULL);
  return 0;
}

void debugController(Control360_T c)
{
fprintf(stderr, "============ DEBUG XBOX CONTROLLER ============\n");
fprintf(stderr, "%d %d %d %d %d %d %d %d %d %d %d %d %d %d\n", 
        c.Pad_Up,
        c.Pad_Down,
        c.Pad_Left,
        c.Pad_Right,
        c.Left_Bump,
        c.Right_Bump,
        c.A_Button,
        c.B_Button,
        c.X_Button,
        c.Y_Button,
        c.Left_X_Val,
        c.Left_Y_Val,
        c.Right_X_Val,
        c.Right_Y_Val);
}
