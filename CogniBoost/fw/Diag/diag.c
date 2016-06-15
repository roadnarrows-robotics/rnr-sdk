/*This file is prepared for Doxygen automatic documentation generation.*/
/*! \file *********************************************************************
 *
 * \brief Main file of the USB CDC example.
 *
 * - Compiler:           IAR EWAVR32 and GNU GCC for AVR32
 * - Supported devices:  All AVR32 devices with a USB module can be used.
 * - AppNote:
 *
 * \author               Atmel Corporation: http://www.atmel.com \n
 *                       Support and FAQ: http://support.atmel.no/
 *
 ******************************************************************************/

/*! \mainpage AVR32 USB software framework for CDC
 *
 * This manual is divided into the following sections:
 * - \ref license "Licence"
 * - \ref desc "Description of the Communication Device Class (CDC) USB class"
 * - \ref sample "About the Sample Application"
 * - \ref device_use "Using the USB Device Mode"
 * - \ref host_use "Using the USB Host Mode"
 * - \ref arch "Firmware Architecture"
 * - \ref files "Main Files used by the demo"
 *
 * <HR>
 * \section license License
 * Use of this program is subject to Atmel's End User License Agreement.
 *
 * Please read the \ref license at the bottom of this page.
 *
 * \section desc Description of the Communication Device Class (CDC) USB class
 * The Communication Device Class (CDC) is a general-purpose way to enable all
 * types of communications on the Universal Serial Bus (USB). This class makes it possible
 * to connect telecommunication devices such as digital telephones or analog
 * modems, as well as networking devices like ADSL or Cable modems.
 * While a CDC device enables the implementation of quite complex devices, it can also
 * be used as a very simple method for communication on the USB. For example, a CDC
 * device can appear as a virtual COM port, which greatly simplifies application programming
 * on the host side.
 *
 * \section sample About the Sample Application
 * This embedded application source code illustrates how to implement a USB CDC application
 * on the AVR32 microcontroller.
 *
 * As the AVR32 implements a device/host USB controller, the embedded application can operate
 * in one of the following USB operating modes:
 *   - USB device;
 *   - USB reduced-host controller;
 *   - USB dual-role device (depending on the ID pin).
 *
 * To optimize embedded code/RAM size and reduce the number of source modules, the application can be
 * configured to use one and only one of these operating modes.
 *
 * \page device_use Using the USB Device Mode
 *
 * The DEVICE source code illustrates how to implement on the AVR32 microcontroller
 * a Serial to USB application.
 *
 * \image html appli.jpg "Serial to USB application architecture"
 *
 * \section install Installation
 *
 * For our example, we will only use a PC for the demonstration: it will both connect to the USB and to the
 * COM port of the EVK110x boards.
 *
 * Connect the USART peripheral to the USART connector of the EVK110x board.
 * Connect the application to a USB host (e.g. a PC) with a mini-B (embedded side) to A (PC host side) cable.
 * The application will behave as a virtual COM.
 * \image html appli_evk1100_device.jpg "EVK1100 USB Device Mode"
 * \image html appli_evk1101_device.jpg "EVK1101 USB Device Mode"
 * For the first connection of the EVK110x board on the PC,
 * the operating system will detect a new peripheral. This will open a new hardware installation window.
 * Choose "No, not this time" to connect to Windows Update for this installation and click `Next':
 *
 * \image html install_1.jpg "Found New Hardware Wizard"
 *
 * On the next screen, select "Install from a list or specific location (Advanced)" and click `Next'.
 * Then request to search in the DRIVER folder of the "CDC EXAMPLES"
 * directory as shown below and click `Next'
 *
 * \image html install_3.jpg "location"
 *
 * A warning message "The software [...]
 * has not passed Windows Logo testing..." will appear. Click on "Continue Anyway":
 *
 * \image html install_4.jpg "Warning"
 *
 * Windows will then process the installation of the driver. Once completed, click `Finish'.
 *
 * \image html install_5.jpg "Installation done"
 *
 * For our example, we will use 2 serial terminals (e.g. HyperTerminal under
 * Windows systems or minicom under Linux systems; USART settings: 57600 bauds, 8-bit data, no parity bit,
 * 1 stop bit, no flow control):
 *  - One on the virtual COM, known as "AT32UC3xxx CDC USB to USART" (known as "Virtual Serial Term" in this document).
 * You will find the virtual COM number in the Windows Device Manager, as shown hereafter (in this example,
 * the virtual COM port number is COM6):
 *
 * \image html usb_cdc_device_manager.jpg "Installation done"
 *
 *  - One on the COM on which the EVK110x serial output has been connected to (known as "Serial Term" in this document).
 *
 * Open the 2 terminals using the same baudrates.
 *  - Any characters typed on the "Virtual Serial COM" terminal will be transferred from USB through USART and thus be dispayed on the "Serial Term"
 *  - Any characters typed on the "Serial COM" terminal will be transferred from USART through USB and then be dispayed on the "Virtual Serial Term"
 *
 * Note that the serial parameters (baud rate, stop bits, etc...) of the "Virtual Serial Term" are automatically
 * applied on the "Serial COM" USART.
 *
 * \page host_use Using the USB Host Mode
 *
 * This embedded application source code illustrates how to implement a simple USB host application with CDC support.
 * In the host mode, the USB can be connected (with a mini A connector attached to the mini AB EVK110x receptacle)
 * to a standard CDC device (or another EVK110x running device CDC application).
 * The controller will initialize and enumerate a device CDC application.
 *
 * In our example, we will connect 2 EVK110x boards together using the USB: one will act as a host, the second
 * one will operate as a device.
 * Connect the USART output of each board to the COM ports of a PC (e.g. HyperTerminal under
 * Windows systems or minicom under Linux systems; USART settings: 57600 bauds, 8-bit data, no parity bit,
 * 1 stop bit, no flow control).
 *
 * \image html appli_host.jpg
 *
 * Open the 2 terminals using the same baudrates.
 *  - Any characters typed on the "Serial COM x" terminal will be transferred through USART and then be dispayed on the "Serial COM y"
 *  - Any characters typed on the "Serial COM y" terminal will be transferred through USART and then be dispayed on the "Serial COM x"
 *
 * The following picture shows the architecture of the host application:
 * \image html appli_host.jpg
 *
 * \page arch Architecture
 * As illustrated in the figure bellow, the application entry point is located is the cdc_example.c file.
 * The application can be ran using the FreeRTOS operating system, or in standalone mode.
 * In the standalone mode, the main function runs the usb_task, the device_cdc_task and host_cdc_task tasks in an infinite loop.
 *
 * The application is based on three different tasks:
 * - The usb_task  (usb_task.c associated source file), is the task performing the USB low level
 * enumeration process in device mode.
 * Once this task has detected that the usb connection is fully operationnal, it updates different status flags
 * that can be checked within the high level application tasks.
 * - The device_cdc_task task performs the high level device application operation.
 * Once the device is fully enumerated (DEVICE SET_CONFIGURATION request received), the task will transform
 * the product as a virtual COM port: any bytes received from the USB will be redirected to the USART and vice versa.
 * - The host_cdc_task task performs the high level host application operation.
 * Once a device is connected and fully enumerated, the task will also transform the product as a virtual COM port.
 * You can for example connect an other EVK110x board itself in the device role.
 *
 * \image html arch_full.jpg "Architecture Overview"
 *
 * \page files Main Files
 * - device_cdc_task.c : high level device CDC application operation
 * - device_cdc_task.h : high level device CDC application operation header
 * - host_cdc_task.c : high level host CDC application operation
 * - host_cdc_task.h : high level host CDC application operation header
 * - uart_usb_lib.c : controls the USART USB functions.
 * - uart_usb_lib.h : USART USB functions header.
 * - cdc_example.c : CDC code example
 *
 * \section license Copyright notice
 * Copyright (c) 2009 Atmel Corporation. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 * list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 *
 * 3. The name of Atmel may not be used to endorse or promote products derived
 * from this software without specific prior written permission.
 *
 * 4. This software may only be redistributed and used in connection with an Atmel
 * AVR product.
 *
 * THIS SOFTWARE IS PROVIDED BY ATMEL "AS IS" AND ANY EXPRESS OR IMPLIED
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT ARE
 * EXPRESSLY AND SPECIFICALLY DISCLAIMED. IN NO EVENT SHALL ATMEL BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE
 *
 */

//_____  I N C L U D E S ___________________________________________________

#include <avr32/io.h>
#if 0 // RDK
#ifndef FREERTOS_USED
#if (defined __GNUC__)
#include "nlao_cpu.h"
#include "nlao_usart.h"
#endif
#else
#include <stdio.h>
#endif
#include "compiler.h"
#include "board.h"
#include "print_funcs.h"
#include "intc.h"
#include "power_clocks_lib.h"
#include "cdc_example.h"
#ifdef FREERTOS_USED
#include "FreeRTOS.h"
#include "task.h"
#endif
#include "conf_usb.h"
#include "usb_task.h"
#if USB_DEVICE_FEATURE == true
#include "device_cdc_task.h"
#endif
#if USB_HOST_FEATURE == true
// BHW #include "host_cdc_task.h"
#endif
#include <stdio.h>
#include "delay.h"                    //BHW
#include "gpio.h"                     //BHW
#include "CogniBoost.h"               //BHW
#include "CogniBoostFwServer.h"       //BHW
#include "CogniBoostFwDataStructs.h"  //BHW
#include "CogniBoostMsgs.h"           //BHW
#include "uart_usb_lib.h"             //BHW
#include "rnr/netmsgs.h"              //BHW
#endif // RDK


//_____ M A C R O S ________________________________________________________


//_____ D E F I N I T I O N S ______________________________________________

/*! \name System Clock Frequencies
 */
//! @{
pcl_freq_param_t pcl_freq_param =
{
  .cpu_f        = APPLI_CPU_SPEED,
  .pba_f        = APPLI_PBA_SPEED,
  .osc0_f       = FOSC0,
  .osc0_startup = OSC0_STARTUP
};
//! @}


#ifndef FREERTOS_USED

  #if (defined __GNUC__)

/*! \brief Low-level initialization routine called during startup, before the
 *         main function.
 *
 * This version comes in replacement to the default one provided by the Newlib
 * add-ons library.
 * Newlib add-ons' _init_startup only calls init_exceptions, but Newlib add-ons'
 * exception and interrupt vectors are defined in the same section and Newlib
 * add-ons' interrupt vectors are not compatible with the interrupt management
 * of the INTC module.
 * More low-level initializations are besides added here.
 */
int _init_startup(void)
{
  // Import the Exception Vector Base Address.
  extern void _evba;

  // Load the Exception Vector Base Address in the corresponding system register.
  Set_system_register(AVR32_EVBA, (int)&_evba);

  // Enable exceptions.
  Enable_global_exception();

  // Initialize interrupt handling.
  INTC_init_interrupts();

  // Initialize the USART used for the debug trace with the configured parameters.
  set_usart_base( ( void * ) DBG_USART );

  // Don't-care value for GCC.
  return 1;
}

  #elif __ICCAVR32__

/*! \brief Low-level initialization routine called during startup, before the
 *         main function.
 */
int __low_level_init(void)
{
  // Enable exceptions.
  Enable_global_exception();

  // Initialize interrupt handling.
  INTC_init_interrupts();

  // Initialize the USART used for the debug trace with the configured parameters.
  extern volatile avr32_usart_t *volatile stdio_usart_base;
  stdio_usart_base = DBG_USART;

  // Request initialization of data segments.
  return 1;
}

  #endif  // Compiler

#endif  // FREERTOS_USED


enum REQUEST_STATES {
  DONE = 0x01,
  NOTDONE = 0x02,
  BAD = 0x04,
};

unsigned char cb_input_buffer[MAX_CB_MSG_SIZE];

unsigned char c;

void ReqInit()
{
	req.state = NOTDONE;
	req.num_bytes = 0;
	req.msg_hdr.magic = 0;
	req.msg_hdr.tid = 0;
	req.msg_hdr.msg_len = 0;
}

bool GetReq(void)
{
  if (req.state == DONE)
	{
	  return true;
	}
	// Magic number wasn't right or something...
	if (req.state == BAD)
	{
		return false;
	}


	while ( uart_usb_test_hit() )
	{
	  c = uart_usb_getchar();
		switch (req.num_bytes)
		{
		  case 0:
			  req.msg_hdr.magic = c;
				break;
      case 1:
			  req.msg_hdr.magic <<= 8;
				req.msg_hdr.magic |= c;
				if (req.msg_hdr.magic != CB_MAGIC_NUM)  // All code assumes that a good magic # means all will be good.
				{
					req.state = BAD;
				}
				break;
		  case 2:
			  req.msg_hdr.tid = c;
				break;
			case 3:
			  req.msg_hdr.msg_len = c;
			  if (req.msg_hdr.msg_len > MAX_CB_MSG_SIZE)
			  {
				  // Don't bother reading a message that is too long.
				  // Something isn't right.
				  req.state = BAD;
			  }

				break;
		  case 4:
			  req.msg_hdr.msg_len <<= 8;
				req.msg_hdr.msg_len |= c;
				break;
			default:
			  if ( req.num_bytes >= MAX_CB_MSG_SIZE + req.msg_hdr.msg_len )
				{
				  req.state = DONE;
				}
				else
				{
				  cb_input_buffer[req.num_bytes-CB_MSG_HDR_SIZE] = c; /*CB_MSG_HDR_LEN*/
				}
			  break;
    }
		++req.num_bytes;
	}
}


/*! \brief Main function. Execution starts here.
 *
 * \retval 42 Fatal error.
 */
int main(void)
{
  // Configure system clocks.
  if (pcl_configure_clocks(&pcl_freq_param) != PASS)
    return 42;



/*************** BHW Board initialization stuff ***************/
	delay_init(APPLI_CPU_SPEED);

	// Don't Put CogniMem in standby immediately
	gpio_configure_pin(STANDBY, GPIO_DIR_OUTPUT | GPIO_INIT_LOW);

	// Turn off, but enable driving of RGB LED
	gpio_configure_pin(LED_TRI0_RED,  GPIO_DIR_OUTPUT | GPIO_INIT_LOW);
	gpio_configure_pin(LED_TRI0_GREEN,GPIO_DIR_OUTPUT | GPIO_INIT_LOW);
	gpio_configure_pin(LED_TRI0_BLUE, GPIO_DIR_OUTPUT | GPIO_INIT_LOW);

	//---- Turn off I2C & other stuff ----
	gpio_configure_pin(I2CEN, GPIO_DIR_OUTPUT | GPIO_INIT_LOW);  // Off
	gpio_configure_pin(RW, GPIO_DIR_OUTPUT | GPIO_INIT_HIGH);    // Default to read
	gpio_configure_pin(RDY, GPIO_DIR_INPUT);
	gpio_configure_pin(DS, GPIO_DIR_OUTPUT | GPIO_INIT_LOW);

  //---- Set address lines as outputs ----
	gpio_configure_pin(REG0,  GPIO_DIR_OUTPUT | GPIO_INIT_LOW);
	gpio_configure_pin(REG1,  GPIO_DIR_OUTPUT | GPIO_INIT_LOW);
	gpio_configure_pin(REG2,  GPIO_DIR_OUTPUT | GPIO_INIT_LOW);
	gpio_configure_pin(REG3,  GPIO_DIR_OUTPUT | GPIO_INIT_LOW);
	gpio_configure_pin(REG4,  GPIO_DIR_OUTPUT | GPIO_INIT_LOW);

	//---- Reset CM1K now that Atmel is now setup to talk to it ----
	gpio_configure_pin(GRESET, GPIO_DIR_OUTPUT | GPIO_INIT_LOW); // Active low
  gpio_set_pin_high(GRESET);  // Low -> High should reset CM1K

	if (readRegister(CM_REG_MINIF_OFF) == CM_REG_MINIF_DFT)  // 0x06 dft val: 0x0002
  {
	  gpio_set_pin_high(LED_TRI0_GREEN);
  }

  if (writeRegister(CM_REG_MINIF_OFF, 0x55) == 0x55)
  {
	  gpio_set_pin_high(LED_TRI0_GREEN);
  }



/*************** NetMsgs stuff ***************/

 /*!
 * \brief Unpack a cbReqReadCMReg ITV message in big-endian byte order
 *        from the input buffer.
 *
 * \param [in] buf        Output message buffer.
 * \param uMsgLen         Length of message (bytes) in input buffer.
 * \param [out] pStruct   Pointer to the associated message structure.
 * \param bTrace          Do [not] trace packing.
 *
 * \return
 * On success, returns the number of bytes unpacked.
 * On error, returns the appropriate \h_lt 0 negated NM_ECODE.
 */

  //int error;
  //uint8_t in_buff[255];
  //cbReqReadCMReg_T register_value;
  /*error = cbUnpackReqReadCMReg( byte_t buf[],
                                      size_t uMsgLen,
                                      cbReqReadCMReg_T * pStruct,
                                      bool_t bTrace ) */


  //int error = cbUnpackReqReadCMReg(in_buff, /*uMsgLen*/10, &register_value, true);

/*!
 * \brief Pack a cbReqReadCMReg ITV message in big-endian byte order
 *        into the output buffer.
 *
 * \param [in] pStruct    Pointer to the associated, populated message
 *                        structure.
 * \param [out] buf       Output message buffer.
 * \param bufSize         Size of output buffer.
 * \param bTrace          Do [not] trace packing.
 *
 * \return
 * On success, returns the number of bytes packed.
 * On error, returns the appropriate \h_lt 0 negated NM_ECODE.


INLINE_IN_H int cbPackReqReadCMReg( cbReqReadCMReg_T * pStruct,
                                    byte_t buf[],
                                    size_t bufSize,
                                    bool_t bTrace ) */

  // Initialize usart comm
  init_dbg_rs232(pcl_freq_param.pba_f);

#ifndef FREERTOS_USED
# if (defined __GNUC__)
  // Give the used CPU clock frequency to Newlib, so it can work properly.
  set_cpu_hz(pcl_freq_param.pba_f);
# endif
#endif

  // Initialize USB clock.
  pcl_configure_usb_clock();

  // Initialize USB task
  usb_task_init();

  // Display a welcome banner on USART
  printf("                                                       ......       ......     \r\n");
  printf("       IIIIII  IIIII        IIII   IIIIIIIIIII      IIIIIIIIIII. .IIIIIIIIII.  \r\n");
  printf("      IIIIIII   IIIII      IIIII  IIIIIIIIIIIII     IIIIIIIIIIII..IIIIIIIIIII. \r\n");
  printf("     IIIIIIIII  IIIII     IIIII   IIIII   IIIII     I.      IIIII.:.     IIIII \r\n");
  printf("     IIII IIIII  IIIII    IIII   IIIII    IIIII            .IIII.        IIIII \r\n");
  printf("    IIIII  IIII   IIII   IIIII  IIIIIIIIIIIIII         IIIIIIII          IIII. \r\n");
  printf("    IIII   IIIII  IIIII IIIII   IIIIIIIIIIIII          IIIIIIII.       .IIII:  \r\n");
  printf("   IIIII    IIIII  IIIIIIIII   IIIIIIIIIII                 .IIIII     IIIII.   \r\n");
  printf("  IIIIIIIIIIIIIII   IIIIIIII   IIIII IIIII                  .IIII   .IIII:     \r\n");
  printf("  IIIIIIIIIIIIIIII  IIIIIII   IIIII   IIII         II:.    .IIIII .IIIII.      \r\n");
  printf(" IIIII        IIIII  IIIIII  IIIII    IIIII        IIIIIIIIIIIII.IIIIIIIIIIIIII\r\n");
  printf(" IIIII        IIIII  IIIII   IIIII    IIIII        :IIIIIIIIII.  IIIIIIIIIIIIII\r\n");
  printf("                      III                                                      \r\n");
  printf("                       II                                                      \r\n");

#if USB_DEVICE_FEATURE == true
  // Initialize device CDC USB task
  device_cdc_task_init();
#endif
#if USB_HOST_FEATURE == true
  // Initialize host CDC USB task
  host_cdc_task_init();
#endif

#ifdef FREERTOS_USED
  // Start OS scheduler
  vTaskStartScheduler();
  portDBG_TRACE("FreeRTOS returned.");
  return 42;
#else
  // No OS here. Need to call each task in round-robin mode.

  /****************************************************************************/
  cb_msg_header msg_hdr;
  cbMsgId_T msg_id;
  /****************************************************************************/

  while (true)
  {
    usb_task();
  #if USB_DEVICE_FEATURE == true
    device_cdc_task();
  #endif
  #if USB_HOST_FEATURE == true
    host_cdc_task();
  #endif

/*******************************************************************************/
  if ( GetReq() )
  {
	  // Get local copy of msg_id & msg_hdr
	  msg_id = cb_input_buffer[0];
	  msg_id <<= 8;
	  msg_id |= cb_input_buffer[1];

	  msg_hdr = req.msg_hdr;

	  switch (msg_id)
	  {
		  cbMsgIdNone:
		  cbMsgIdNumOf:
		    break;
		  cbMsgIdReqLoopback:

		    break;
		  cbMsgIdReqReadCMReg:
		  cbMsgIdReqWriteCMReg:
		  cbMsgIdRspErr:
		  cbMsgIdRspLoopback:
		  cbMsgIdRspOk:
		  cbMsgIdRspReadCMReg:
		    break;
		  default:
		    break;
	  }
  }
/*******************************************************************************/

  }
#endif  // FREERTOS_USED
}
