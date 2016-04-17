////////////////////////////////////////////////////////////////////////////////
//
// Package:   Hekateros
//
// Firmware:  Encoder
//
// File:      encoder.c
//
/*! \file
 *
 * $LastChangedDate: 2010-02-27 15:33:08 -0700 (Sat, 27 Feb 2010) $
 * $Rev: 278 $
 *
 * \brief Hekateros Encoder firmware.
 *
 * This program contains the firmware for the RoadNarrows Hekateros Encoder.
 * The module operates on an \h_i2c (aka: 2-wire or TWI) interface and provides
 * data in both raw and ??calibrated form.
 *
 * \todo
 * Finish the code?
 *
 * \see Atmel AtMEGA168\n
 * (http://www.atmel.com/dyn/resources/prod_documents/doc8271.pdf)
 *
 * \author Casey Kuhns (casey@roadnarrows.com)
 * \author Robin Knight (robin.knight@roadnarrows.com)
 *
 * \par Copyright
 * (C) 2006-2009.  RoadNarrows LLC.
 * (http://www.roadnarrows.com)
 * \n All Rights Reserved
 */
////////////////////////////////////////////////////////////////////////////////


#define F_CPU 1000000UL   ///< 1MHz Clock Speed (for delay.h)

#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/eeprom.h>
#include <util/delay.h>
#include <inttypes.h>
#include <stdio.h>
#include <stdlib.h>

#include "Hekateros/encoder.h"

#include "TWI_slave.h"


// .............................................................................
// Defines
// .............................................................................

//
// Address Resolution Protocol (not fully implemented)
//
#define ARP_DEVICE_CAPABILITIES   0x00        ///< device capabilities
#define ARP_VERSION_REVISION      0x02        ///< firmware version (bump)
#define ARP_VENDOR_ID             0x0000      ///< vendor id
#define ARP_DEVICE_ID             ENCODER_DEVICE_ID 
                                              ///< RoadNarrows SZ device id
#define ARP_INTERFACE             0x0000      ///< interface
#define ARP_SUBSYSTEM_VENDOR_ID   0x0000      ///< subsystem vendor id
#define ARP_SUBSYSTEM_DEVICE_ID   0x0000      ///< subsystem device id
#define ARP_VENDOR_SPECIFIC_ID    0x00000000  ///< additional info

//
// 512 Byte EEPROM Configuration Database Layout
//
#define EEPROM_ADDR_MAGIC           0x0000    ///< 2 byte magic number
#define EEPROM_ADDR_I2C_ADDR        0x0002    ///< 1 byte \h_i2c address
#define EEPROM_ADDR_CAL_ZERO_PT     0x0003    ///< 1 byte \h_i2c address
#define EEPROM_ADDR_LED_EN          0x0005    ///< 1 byte enable LED auto ctl
#define EEPROM_ADDR_LED_FIXED       0x0006    ///< 1 byte fixed LED pattern
#define EEPROM_ADDR_LED_BLING       0x0007    ///< 1 byte auto bling behavior
#define EEPROM_ADDR_LED_BLING_TH_LO 0x0008    ///< 2 byte LED bling low threshld
#define EEPROM_ADDR_LED_BLING_TH_HI 0x000a    ///< 2 byte LED bling high thrshld
#define EEPROM_ADDR_CRC             0x000c    ///< 2 byte CRC
#define EEPROM_ADDR_MAX             0x01ff    ///< maximum EEPROM address

#define EEPROM_CFG_LEN (EEPROM_ADDR_CRC+2)    ///< byte length of config db

#define EEPROM_VAL_MAGIC            0xface    ///< 2 byte magic number value

// .............................................................................
// Prototypes
// .............................................................................

void ledSet(uint8_t);


// .............................................................................
// Global Variable Definitions and Defaults
// .............................................................................

/*!
 * \h_i2c address.
 */
uint8_t I2CAddr = ENCODER_ADDR_DFT;



// .............................................................................
// Macro and Function Utilities
// .............................................................................

/*!
 * \brief Validate \h_i2c address.
 *
 * \param addr  8-bit address.
 */
#define IS_VALID_I2C_ADDR(addr)     (((addr) >= 0x08) && ((addr) <= 0x77))

/*!
 * \brief Validate EEPROM address.
 *
 * \param addr  EEPROM value.
 */
#define IS_VALID_EEPROM_ADDR(addr)  ((addr) <= EEPROM_ADDR_MAX)

/*!
 * \brief Pack 16-bit value into buffer.
 *
 * \param uVal        Value to pack.
 * \param [out] buf   Buffer to containod packed bytes.
 */
static inline void pack16(uint16_t uVal, uint8_t buf[])
{
  buf[0] = (uint8_t)(uVal >> 8);
  buf[1] = (uint8_t)(uVal & 0xff);
}

/*!
 * \brief Unack 16-bit value from buffer.
 *
 * \param [in] buf    Buffer to containod packed bytes.
 *
 * \return 16-bit unsigned short.
 */
static inline uint16_t unpack16(uint8_t buf[])
{
  return (uint16_t)((((uint16_t)buf[0]) << 8) | ((uint16_t)(buf[1]) & 0xff));
}


// .............................................................................
//  Working State Variables Functions
// .............................................................................

/*!
 * \brief Validate and set I2C address working variable.
 *
 * \param i2cAddr   New I2C address.
 *
 * \return
 * Returns \ref ENCODER_RSP_OK on success, \ref ENCODER_RSP_FAIL on failure.
 */
uint8_t varSetI2CAddr(uint8_t i2cAddr)
{
  // set working I2C address
  if( IS_VALID_I2C_ADDR(i2cAddr) )
  {
    I2CAddr = i2cAddr;
    return ENCODER_RSP_OK;
  }
  else
  {
    return ENCODER_RSP_FAIL;
  }
}

// .............................................................................
//  EEPROM Functions
// .............................................................................

/*!
 * \brief Write 8-bits to EEPROM.
 *
 * \param eepromAddr  EEPROM address [0, EEPROM_ADDR_MAX].
 * \param byVal       Value to write.
 */
void eepromWrite8(uint16_t eepromAddr, uint8_t byVal)
{
  uint8_t bySREG;

  // store SREG value
  bySREG = SREG;

  // disable interrupts during timed sequence
  cli();

  // write byte
  eeprom_write_byte((uint8_t *)eepromAddr, byVal);

  // restore SREG value (I-bit)
  SREG = bySREG;
}

/*!
 * \brief Read 8-bits from EEPROM.
 *
 * \param eepromAddr  EEPROM address [0, EEPROM_ADDR_MAX].
 *
 * \return Value read.
 */
uint8_t eepromRead8(uint16_t eepromAddr)
{
  uint8_t bySREG;
  uint8_t byVal;

  // store SREG value
  bySREG = SREG;

  // disable interrupts during timed sequence
  cli();

  // return data from data register
  byVal = eeprom_read_byte((uint8_t *)eepromAddr);

  // restore SREG value (I-bit)
  SREG = bySREG;

  return byVal;
}

/*!
 * \brief Write 16-bits to EEPROM.
 *
 * \param eepromAddr  EEPROM address [0, EEPROM_ADDR_MAX].
 * \param huVal       Value to write.
 */
void eepromWrite16(uint16_t eepromAddr, uint16_t huVal)
{
  eepromWrite8(eepromAddr,    (uint8_t)(huVal >> 8));
  eepromWrite8(eepromAddr+1,  (uint8_t)(huVal & 0xff));
}

/*!
 * \brief Read 16-bits from EEPROM.
 *
 * \param eepromAddr  EEPROM address [0, EEPROM_ADDR_MAX].
 *
 * \return Value read.
 */
uint16_t eepromRead16(uint16_t eepromAddr)
{
  uint16_t  huVal;

  huVal  = eepromRead8(eepromAddr) << 8;
  huVal |= eepromRead8(eepromAddr+1);

  return huVal;
}


// .............................................................................
//  Configuration Database Functions
// .............................................................................

/*!
 * \brief Caculcate the EEPROM database CRC.
 * \return Calculated CRC.
 */
uint16_t dbCalcCRC(void)
{
  uint16_t  addr;
  uint16_t  crc;

  // calculate crc over eeprom data excluding the magic number and the crc itself
  for(addr=EEPROM_ADDR_I2C_ADDR, crc=0; addr<EEPROM_CFG_LEN-2; ++addr)
  {
    crc += (uint16_t)eepromRead8(addr);
  }
  return crc;
}

/*!
 * \brief Mark database as valid.
 */
void dbMarkValid(void)
{
  uint16_t  crc;

  // add crc to db
  crc = dbCalcCRC();
  if( eepromRead16(EEPROM_ADDR_CRC) != crc )
  {
    eepromWrite16(EEPROM_ADDR_CRC, crc);
  }

  // lay some magic
  if( eepromRead16(EEPROM_ADDR_MAGIC) != EEPROM_VAL_MAGIC )
  {
    eepromWrite16(EEPROM_ADDR_MAGIC, EEPROM_VAL_MAGIC);
  }
}

/*!
 * \brief Check if database if valid.
 * \return 1 if valid, else 0.
 */
uint8_t dbIsValid(void)
{
  if( eepromRead16(EEPROM_ADDR_MAGIC) != EEPROM_VAL_MAGIC )
  {
    return 0;
  }
  else if( eepromRead16(EEPROM_ADDR_CRC) != dbCalcCRC() )
  {
    return 0;
  }
  else
  {
    return 1;
  }
}

/*!
 * \brief Write 8-bits to EEPROM database. Database integrity is maintained.
 *
 * \param eepromAddr  EEPROM address [0, EEPROM_ADDR_MAX].
 * \param byVal       Value to write.
 */
void dbWrite8(uint16_t eepromAddr, uint8_t byVal)
{
  eepromWrite8(eepromAddr, byVal);
  dbMarkValid();
}

/*!
 * \brief Write 16-bits to EEPROM database. Database integrity is maintained.
 *
 * \param eepromAddr  EEPROM address [0, EEPROM_ADDR_MAX].
 * \param huVal       Value to write.
 */
void dbWrite16(uint16_t eepromAddr, uint16_t huVal)
{
  eepromWrite16(eepromAddr, huVal);
  dbMarkValid();
}

//TODO  FIX THIS!!!
/*!
 * \brief Store working configuration to EEPROM.
 */
void dbStore(void)
{
 /* eepromWrite8(EEPROM_ADDR_I2C_ADDR, I2CAddr);
  eepromWrite16(EEPROM_ADDR_CAL_ZERO_PT, (uint16_t)StrainZeroPt);
  eepromWrite8(EEPROM_ADDR_LED_EN, LEDBlingEnable);
  eepromWrite8(EEPROM_ADDR_LED_FIXED, LEDBlingFixed);
  eepromWrite8(EEPROM_ADDR_LED_BLING, LEDBling);
  eepromWrite16(EEPROM_ADDR_LED_BLING_TH_LO, LEDBlingThTension);
  eepromWrite16(EEPROM_ADDR_LED_BLING_TH_HI, LEDBlingThCompress);
*/
  dbMarkValid();
}
//TODO  FIX THIS!!!
/*!
 * \brief Load working configuration from EEPROM.
 */
void dbLoad(void)
{
  // RDK TODO
  return;
  //
  // If uninitialized or corrupted database, use defaults.
  //
  if( !dbIsValid() )
  {
    dbStore();   // save defaults into eeprom
    return;
  }

  //
  // Load working state from database.
  //
  
  varSetI2CAddr(eepromRead8(EEPROM_ADDR_I2C_ADDR));

  /*StrainZeroPt    = (int16_t)eepromRead16(EEPROM_ADDR_CAL_ZERO_PT);

  LEDBlingEnable  = eepromRead8(EEPROM_ADDR_LED_EN);
  LEDBlingFixed   = eepromRead8(EEPROM_ADDR_LED_FIXED);

  ledSet(LEDBlingFixed);

  varSetLEDBling(eepromRead8(EEPROM_ADDR_LED_BLING));
  
  varSetLEDBingThreshold(eepromRead16(EEPROM_ADDR_LED_BLING_TH_LO),
                         eepromRead16(EEPROM_ADDR_LED_BLING_TH_LO));*/
}


// .............................................................................
// Two Wire Interface (I2C) Functions
// .............................................................................

/*!
 * \brief Initialized \h_i2c Two-Wire Interface.
 */
void twiInit(void)
{
  //
  // Initialize TWI module for slave operation. Include address and/or enable
  // General Call.
  //
  TWI_Slave_Initialise((I2CAddr<<TWI_ADR_BITS) | (TRUE<<TWI_GEN_BIT)); 
  
  //
  // Start the TWI transceiver to enable reseption of the first command from
  // the TWI Master.
  //
  TWI_Start_Transceiver();
}

/*!
 * \brief Handle Two-Wire Interface error.
 * 
 * The LEDs are flashed with the error code pattern.
 * The interface transceiver is restarted with all of the same data in the
 * transmission buffers.
 *
 * \param TWIerrorMsg   Error code.
 * 
 * \return Error code.
 */
unsigned char twiActOnFailureInLastTransmission(unsigned char TWIerrorMsg)
{
  PORTB = TWIerrorMsg;
  TWI_Start_Transceiver();
                    
  return TWIerrorMsg; 
}

/*!
 * \brief Set new \h_i2c address.
 * 
 * The Two-Wire Interface is reset with the new address after the response
 * message is transmitted. The connecting host application must now use the
 * new address for communication.
 *
 * The new setting(s) are saved in EEPROM and are preserved over power
 * disruptions.
 *
 * \par Syntax:
 * cmd: cmdid addr\n
 * rsp: ok | fail
 *
 * \param [in,out] msgbuf Comand/response message buffer.
 * 
 * \return
 * Returns \ref SZHAND_RSP_OK on success, \ref SZHAND_RSP_FAIL on error.
 */
int8_t twiCmdSetI2CAddr(uint8_t msgbuf[])
{
  uint8_t   pf;   // pass/fail

  // set working state
  pf = varSetI2CAddr(msgbuf[1]);

  if( pf == ENCODER_RSP_OK )
  {
    dbWrite8(EEPROM_ADDR_I2C_ADDR, I2CAddr);
  }

  // pack and send repsonse
  msgbuf[0] = pf;
  TWI_Start_Transceiver_With_Data(msgbuf, 1);

  // wait for response to be sent
  while( TWI_Transceiver_Busy() );

  // reset I2C interface with new address
  twiInit();

  return pf;
}

/*!
 * \brief Dump EEPROM memory range \h_i2c command.
 * 
 * \par Syntax:
 * cmd: cmdid addr_hi addr_lo len \n
 * rsp: byte[0] ... byte[len-1]
 *
 * \param [in,out] msgbuf Comand/response message buffer.
 * 
 * \return
 * Returns \ref SZHAND_RSP_OK on success, \ref SZHAND_RSP_FAIL on error.
 */
uint8_t twiCmdDumpEEPROM(uint8_t msgbuf[])
{
  uint16_t  eepromAddr;
  uint8_t   len;
  uint8_t   n;

  eepromAddr  = unpack16(&msgbuf[1]);
  len         = msgbuf[3];

  // dump eeprom location
  for(n=0; n<len; ++n)
  {
    msgbuf[n] = eepromRead8(eepromAddr++);
  }

  // send response
  TWI_Start_Transceiver_With_Data(msgbuf, len);

  return ENCODER_RSP_OK;
}
//TODO FINISH THIS!!
/*!
 * \brief Process Two-Wire Interface \h_i2c command.
 *
 * Each command has a command id in the first byte to identify the command.
 */
void twiProcessCmd(void)
{
  uint8_t   msgbuf[ENCODER_MAX_MSG_SIZE];
  int16_t   hVal;

  // Check if the last operation was successful
  if( TWI_statusReg.lastTransOK )
  {
    // Check if the last operation was a reception
    if( TWI_statusReg.RxDataInBuf )
    {
      // read command
      TWI_Get_Data_From_Transceiver(msgbuf, ENCODER_MAX_CMD_SIZE);         

      // Check if the last operation was a reception as General Call
      if( TWI_statusReg.genAddressCall )
      {
        PORTD = msgbuf[0];  // Put data received to PORTB as an example.
      }

      // Ends up here if the last operation was a reception as
      // Slave Address Match
      else                     
      {
        switch( msgbuf[0] )
        {
          //TODO LIST CASES AND FUNCTIONS
		  //
          // Set Hekateros Encoder I2C address
          // Command:   cmdid addr
          // Response:  ok | fail
          //
          case ENCODER_CMD_SET_I2C_ADDR:
            twiCmdSetI2CAddr(msgbuf);
            break;


          //
          // Get firmware version number.
          // Command:   cmdid 
          // Response:  fwver
          //
          case ENCODER_CMD_GET_FW_VERSION:
            msgbuf[0] = ARP_VERSION_REVISION;
            TWI_Start_Transceiver_With_Data(msgbuf, 1);
            break;

          //
          // Get Hekateros device id.
          // Command:   cmdid 
          // Response:  devid_hi devid_lo
          //
          case ENCODER_CMD_GET_DEVICE_ID:
            pack16(ARP_DEVICE_ID, msgbuf);
            TWI_Start_Transceiver_With_Data(msgbuf, 2);
            break;

          //
          // Dump EEPROM bytes.
          // Command:   cmdid addr_hi addr_lo len
          // Response:  eeprom[0] ... eeprom[len-1]
          //
          case ENCODER_CMD_DUMP_EEPROM:
            twiCmdDumpEEPROM(msgbuf);
            break;

          //
          // Ignore unknown messages.
          //
          default:
            break;
        }
      }
    }

    // Ends up here if the last operation was a transmission
    else
    {
      asm volatile ("nop"::);
    }
  
    // Check if the TWI Transceiver has already been started.
    // If not then restart it to prepare it for new receptions.             
    if( !TWI_Transceiver_Busy() )
    {
      TWI_Start_Transceiver();
    }      
  }
      
  // End up here if the last operation completed unsuccessfully
  else
  {
    twiActOnFailureInLastTransmission( TWI_Get_State_Info() );
  }
}

/*!
 * \brief Initialize ATmega168 ports.
 */
void initPorts(void)
{
  DDRB  = 0xFF;   // debugging port
  DDRD  = 0x00;   // Set to input for encoder channels
  PORTB = 0x00;   // clear
  PORTD = 0x00;   // turn off LEDs
  
  DDRC = 0XFF;
}

/*!
 * \brief Initialize Encoder and settings.
 */
void encoderInit(void){
	//TODO 
	//init encoders here!
}


/*!
 * \brief Initialize the ATMega168 for the Encoder.
 */
void init(void)
{
  // intialize I/O ports
  initPorts();

  // load configuration from eeprom
  dbLoad();

  // initialize I2C
  twiInit();
  
  encoderInit();

  // set global interupts enable.
  sei();
}

/*!
 * \brief Firmware main.
 * 
 * Loop forever reading peripherals and processing commands.
 *
 * \return 0
 */
int main(void)
{
uint8_t i = 0; // RDK
  init();

//_delay_ms(1000); // RDK
 
  //
  // This loop runs forever. If the TWI is busy the execution will just
  // continue doing other operations.
  //
  for(;;)
  {
    // Check if the TWI Transceiver has completed an operation.
//    if( !TWI_Transceiver_Busy() )
//    {
//      twiProcessCmd();
//    }
// RDK


PORTC = 0xFF;
//PORTD = i++;

PORTC = 0x00;

// RDK
  }
  
  return 0;
}
