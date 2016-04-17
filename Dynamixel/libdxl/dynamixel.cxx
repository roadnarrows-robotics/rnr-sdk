////////////////////////////////////////////////////////////////////////////////
//
// Package:   Dynamixel
//
// Library:   libdxl
//
// File:      dynamixel.cxx
//
/*! \file
 *
 * \brief Dynamixel SDK interface.
 *
 * Based on Robotis Inc. dxl_sdk-1.01
 *
 * \author Robotis
 * \author Robin Knight (robin.knight@roadnarrows.com)
 */
/*
 * This file is a modified version of the file freely provided by Robotis Inc.
 * No restrictions by Robotis Inc. nor RoadNarrows LLC are made.
 */
////////////////////////////////////////////////////////////////////////////////

#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "rnr/rnrconfig.h"
#include "rnr/log.h"

#include "Dynamixel/dxl/dxl.h"

#include "dxlhal.h"

using namespace libdxl;

dxl::dxl()
{
  memset(m_bufInstructionPacket, 0, sizeof(m_bufInstructionPacket));
  memset(m_bufStatusPacket, 0, sizeof(m_bufStatusPacket));

  m_nRxPacketLength = 0;
  m_nRxGetLength    = 0;
  m_nCommStatus     = DXL_COMM_RXSUCCESS;
  m_bBusUsing       = false;

  m_fnEnableTx  = NULL;
  m_fnEnableRx  = NULL;
  m_pTxRxArg    = NULL;

  m_pDxlHal = new dxlhal(); 
}

dxl::~dxl()
{
  delete m_pDxlHal;
}

int dxl::open(const char *deviceName, int baudrate)
{
  if( m_pDxlHal->open(deviceName, baudrate) == 0 )
  {
    return 0;
  }

  m_nCommStatus = DXL_COMM_RXSUCCESS;
  m_bBusUsing   = false;

  return 1;
}

int dxl::dxl_initialize(int deviceIndex, int baudnum)
{
  float baudrate;  

  baudrate = dxl_baudnum_to_baudrate(baudnum);

  if( m_pDxlHal->dxl_hal_open(deviceIndex, baudrate) == 0 )
  {
    return 0;
  }

  m_nCommStatus = DXL_COMM_RXSUCCESS;
  m_bBusUsing   = false;

  return 1;
}

void dxl::close()
{
  m_pDxlHal->close();
}

int dxl::setBaudRate(int baudrate)
{
  return m_pDxlHal->setBaudRate(baudrate);
}

int dxl::setHalfDuplexCallbacks(dxl_hdctl_tx_cb_t  enable_tx_cb,
                                dxl_hdctl_rx_cb_t  enable_rx_cb,
                                void              *arg)
{
  m_fnEnableTx  = enable_tx_cb;
  m_fnEnableRx  = enable_rx_cb;
  m_pTxRxArg    = arg;

  return 1;
}

int dxl::getFd()
{
  return m_pDxlHal->getFd();
}

void dxl::setTxPacketId(int id)
{
  m_bufInstructionPacket[DXL_ID] = (uint8_t)id;
}

void dxl::setTxPacketInstruction(int instruction)
{
  m_bufInstructionPacket[DXL_INSTRUCTION] = (uint8_t)instruction;
}

void dxl::setTxPacketParameter(int index, int value)
{
  m_bufInstructionPacket[DXL_PARAMETER+index] = (uint8_t)value;
}

void dxl::setTxPacketLength(int length)
{
  m_bufInstructionPacket[DXL_LENGTH] = (uint8_t)length;
}

unsigned int dxl::getRxPacketErrBits()
{
  return (unsigned int)m_bufStatusPacket[DXL_ERRBIT];
}

bool dxl::getRxPacketError(unsigned int errbit)
{
  return m_bufStatusPacket[DXL_ERRBIT] & errbit? true: false;
}

int dxl::getRxPacketLength()
{
  return (int)m_bufStatusPacket[DXL_LENGTH];
}

int dxl::getRxPacketParameter(int index)
{
  return (int)m_bufStatusPacket[DXL_PARAMETER+index];
}

int dxl::makeWord(int lowbyte, int highbyte)
{
  uint16_t word;

  word = (uint16_t)highbyte;
  word = (uint16_t)(word << 8);
  word = (uint16_t)(word + (uint16_t)lowbyte);
  return (int)word;
}

int dxl::getLowByte(int word)
{
  uint16_t temp;

  temp = (uint16_t)(word & 0xff);
  return (int)temp;
}

int dxl::getHighByte(int word)
{
  uint16_t temp;

  temp = (uint16_t)(word & 0xff00);
  temp = (uint16_t)(temp >> 8);
  return (int)temp;
}

void dxl::txPacket()
{
  int     txInstLen;    ///< length field value in instruction packate
  int     txPktLen;     ///< length of packet to transmit
  int     txLen;        ///< actual length transmitted
  int     i;            ///< working index
  uint8_t checksum = 0; ///< packet checksum

  // in use
  if( m_bBusUsing )
  {
    return;
  }
  
  m_bBusUsing = true;

  txInstLen = (int)m_bufInstructionPacket[DXL_LENGTH];

  if( txInstLen > (DXL_MAXNUM_TXPARAM+2) )
  {
    m_nCommStatus = DXL_COMM_TXERROR;
    m_bBusUsing   = false;
    return;
  }
  
  if(  m_bufInstructionPacket[DXL_INSTRUCTION] != DXL_INST_PING
    && m_bufInstructionPacket[DXL_INSTRUCTION] != DXL_INST_READ
    && m_bufInstructionPacket[DXL_INSTRUCTION] != DXL_INST_WRITE
    && m_bufInstructionPacket[DXL_INSTRUCTION] != DXL_INST_REG_WRITE
    && m_bufInstructionPacket[DXL_INSTRUCTION] != DXL_INST_ACTION
    && m_bufInstructionPacket[DXL_INSTRUCTION] != DXL_INST_RESET
    && m_bufInstructionPacket[DXL_INSTRUCTION] != DXL_INST_SYNC_WRITE )
  {
    m_nCommStatus = DXL_COMM_TXERROR;
    m_bBusUsing   = false;
    return;
  }
  
  m_bufInstructionPacket[0] = 0xff;
  m_bufInstructionPacket[1] = 0xff;

  for( i=0; i<(txInstLen+1); i++ )
  {
    checksum += m_bufInstructionPacket[i+2];
  }
  m_bufInstructionPacket[txInstLen+3] = (uint8_t)~checksum;
  
  if( m_nCommStatus == DXL_COMM_RXTIMEOUT ||
      m_nCommStatus == DXL_COMM_RXCORRUPT )
  {
    m_pDxlHal->clear();
  }

  txPktLen = txInstLen + 4;

  // enable transmit
  if( m_fnEnableTx != NULL )
  {
    m_fnEnableTx(m_pTxRxArg);
  }

  txLen = (uint8_t)m_pDxlHal->tx((uint8_t*)m_bufInstructionPacket, txPktLen);

  if( txPktLen != txLen )
  {
    LOGERROR("Tx failed.");
    m_nCommStatus = DXL_COMM_TXFAIL;
    m_bBusUsing   = false;
    return;
  }

  // enable receive
  if( m_fnEnableRx != NULL )
  {
    m_fnEnableRx(m_pTxRxArg, txPktLen);
  }

  if( m_bufInstructionPacket[DXL_INSTRUCTION] == DXL_INST_READ )
  {
    m_pDxlHal->setTimeout( m_bufInstructionPacket[DXL_PARAMETER+1] + 6 );
  }
  else
  {
    m_pDxlHal->setTimeout( 6 );
  }

  m_nCommStatus = DXL_COMM_TXSUCCESS;
}

void dxl::rxPacket()
{
  int     txPktLen;       // transmitted packet length
  int     nRead;          // number bytes read
  int     i, j;           // working indices
  uint8_t checksum = 0;   // packet checksum

  // no pending (good) transmission
  if( !m_bBusUsing )
  {
    return;
  }

  // no reply on broadcast
  if( m_bufInstructionPacket[DXL_ID] == DXL_BROADCAST_ID )
  {
    m_nCommStatus  = DXL_COMM_RXSUCCESS;
    m_bBusUsing    = false;
    return;
  }
  
  // fixed packet header size
  else if( m_nCommStatus == DXL_COMM_TXSUCCESS )
  {
    m_nCommStatus      = DXL_COMM_RXWAITING;
    m_nRxGetLength    = 0;
    m_nRxPacketLength = 6;
  }
  
  txPktLen = (int)m_bufInstructionPacket[DXL_LENGTH] + 4;

  // enable receive if not already done so
  if( m_fnEnableRx != NULL )
  {
    m_fnEnableRx(m_pTxRxArg, txPktLen);
  }

  //
  // Read fixed packet header.
  //
  nRead = m_pDxlHal->rx((uint8_t*)&m_bufStatusPacket[m_nRxGetLength],
                              m_nRxPacketLength - m_nRxGetLength);

  m_nRxGetLength += nRead;

  // fprintf(stderr, "DBG: read=%d, total=%d\n", nRead, m_nRxGetLength);

  if( m_nRxGetLength < m_nRxPacketLength )
  {
    if( m_pDxlHal->hasTimedOut() )
    {
      // nada
      if( m_nRxGetLength == 0 )
      {
        m_nCommStatus = DXL_COMM_RXTIMEOUT;
        // ping is used to probe the chain for servos - so timeouts are common
        if( m_bufInstructionPacket[DXL_INSTRUCTION] != DXL_INST_PING )
        {
          // TO MANY ERRORS LOGERROR("Rx Timed out with 0 bytes.");
        }
      }
      else
      {
        m_nCommStatus = DXL_COMM_RXCORRUPT;
        LOGERROR("Partial packet of %d bytes received.", m_nRxGetLength);
#if 0 // debug
        for(i=0; i<m_nRxGetLength; ++i)
        {
          fprintf(stderr, "0x%02x ", m_bufStatusPacket[i]);
        }
        fprintf(stderr, "\n");
#endif 
      }
      m_bBusUsing = false;
    }
    return;
  }
  
  //
  // Find packet frame.
  //
  for(i=0; i<(m_nRxGetLength-1); ++i)
  {
    if( m_bufStatusPacket[i] == 0xff && m_bufStatusPacket[i+1] == 0xff )
    {
      break;
    }
    else if( i == m_nRxGetLength-2 &&
              m_bufStatusPacket[m_nRxGetLength-1] == 0xff )
    {
      break;
    }
  }  

  // frame slipped
  if( i > 0 )
  {
    // copy
    for( j=0; j<(m_nRxGetLength-i); j++ )
    {
      m_bufStatusPacket[j] = m_bufStatusPacket[j + i];
    }
      
    m_nRxGetLength -= i;    
  }

  if( m_nRxGetLength < m_nRxPacketLength )
  {
    m_nCommStatus = DXL_COMM_RXWAITING;
    return;
  }

  // Check id pairing
  if( m_bufInstructionPacket[DXL_ID] != m_bufStatusPacket[DXL_ID])
  {
    m_nCommStatus = DXL_COMM_RXCORRUPT;
    m_bBusUsing   = false;
    LOGERROR("Tx/Rx ids mismatch.");
    return;
  }
  
  // expected packet length
  m_nRxPacketLength = (int)m_bufStatusPacket[DXL_LENGTH] + 4;

  //
  // Read remaining variable length packet component.
  //
  if( m_nRxGetLength < m_nRxPacketLength )
  {
    nRead = m_pDxlHal->rx((uint8_t*)&m_bufStatusPacket[m_nRxGetLength],
                                m_nRxPacketLength - m_nRxGetLength );
    m_nRxGetLength += nRead;
    if( m_nRxGetLength < m_nRxPacketLength )
    {
      m_nCommStatus = DXL_COMM_RXWAITING;
      return;
    }
  }

  //
  // Validate checksum.
  //
  for( i=0; i<(m_bufStatusPacket[DXL_LENGTH]+1); i++ )
  {
    checksum += m_bufStatusPacket[i+2];
  }
  checksum = (uint8_t)~checksum;

  if( m_bufStatusPacket[m_bufStatusPacket[DXL_LENGTH]+3] != checksum )
  {
    m_nCommStatus = DXL_COMM_RXCORRUPT;
    m_bBusUsing   = false;
    LOGERROR("Checksum mismatch.");
    return;
  }
  
  m_nCommStatus = DXL_COMM_RXSUCCESS;
  m_bBusUsing   = false;
}

void dxl::txrxPacket()
{
  txPacket();

  if( m_nCommStatus != DXL_COMM_TXSUCCESS )
  {
    return;  
  }
  
  do
  {
    rxPacket();    
  } while( m_nCommStatus == DXL_COMM_RXWAITING );  
}

int dxl::getResult()
{
  return m_nCommStatus;
}

void dxl::ping(int id)
{
  while( m_bBusUsing );

  m_bufInstructionPacket[DXL_ID] = (uint8_t)id;
  m_bufInstructionPacket[DXL_INSTRUCTION] = DXL_INST_PING;
  m_bufInstructionPacket[DXL_LENGTH] = 2;
  
  txrxPacket();
}

int dxl::readByte(int id, int address)
{
  while( m_bBusUsing );

  m_bufInstructionPacket[DXL_ID] = (uint8_t)id;
  m_bufInstructionPacket[DXL_INSTRUCTION] = DXL_INST_READ;
  m_bufInstructionPacket[DXL_PARAMETER] = (uint8_t)address;
  m_bufInstructionPacket[DXL_PARAMETER+1] = 1;
  m_bufInstructionPacket[DXL_LENGTH] = 4;
  
  txrxPacket();

  return (int)m_bufStatusPacket[DXL_PARAMETER];
}

void dxl::writeByte( int id, int address, int value )
{
  while( m_bBusUsing );

  m_bufInstructionPacket[DXL_ID] = (uint8_t)id;
  m_bufInstructionPacket[DXL_INSTRUCTION] = DXL_INST_WRITE;
  m_bufInstructionPacket[DXL_PARAMETER] = (uint8_t)address;
  m_bufInstructionPacket[DXL_PARAMETER+1] = (uint8_t)value;
  m_bufInstructionPacket[DXL_LENGTH] = 4;
  
  txrxPacket();
}

int dxl::readWord( int id, int address )
{
  while( m_bBusUsing );

  m_bufInstructionPacket[DXL_ID] = (uint8_t)id;
  m_bufInstructionPacket[DXL_INSTRUCTION] = DXL_INST_READ;
  m_bufInstructionPacket[DXL_PARAMETER] = (uint8_t)address;
  m_bufInstructionPacket[DXL_PARAMETER+1] = 2;
  m_bufInstructionPacket[DXL_LENGTH] = 4;
  
  txrxPacket();

  return makeWord((int)m_bufStatusPacket[DXL_PARAMETER], (int)m_bufStatusPacket[DXL_PARAMETER+1]);
}

void dxl::writeWord( int id, int address, int value )
{
  while( m_bBusUsing );

  m_bufInstructionPacket[DXL_ID] = (uint8_t)id;
  m_bufInstructionPacket[DXL_INSTRUCTION] = DXL_INST_WRITE;
  m_bufInstructionPacket[DXL_PARAMETER] = (uint8_t)address;
  m_bufInstructionPacket[DXL_PARAMETER+1] = (uint8_t)getLowByte(value);
  m_bufInstructionPacket[DXL_PARAMETER+2] = (uint8_t)getHighByte(value);
  m_bufInstructionPacket[DXL_LENGTH] = 5;
  
  txrxPacket();
}

float dxl::dxl_baudnum_to_baudrate(int baudnum)
{
  switch( baudnum )
  {
    case DXL_BAUDNUM_EXT_2250000:
      return 2250000.0f;
    case DXL_BAUDNUM_EXT_2500000:
      return 2500000.0f;
    case DXL_BAUDNUM_EXT_3000000:
      return 3000000.0f;
    default:
      return 2000000.0f / (float)(baudnum + 1);
  }
}
