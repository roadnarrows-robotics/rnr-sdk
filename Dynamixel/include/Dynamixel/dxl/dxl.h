////////////////////////////////////////////////////////////////////////////////
//
// Package:   Dynamixel
//
// Library:   libdxl
//
// File:      dxl.h
//
/*! \file
 *
 * \brief Modified dynamixel SDK interface.
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

#ifndef _DXL_H
#define _DXL_H

#include <sys/types.h>
#include <stdint.h>

#include <string>

namespace libdxl
{
  //----------------------------------------------------------------------------
  // Defines and Types
  //----------------------------------------------------------------------------

  //
  // Common baud number enumeration.
  //
#define DXL_BAUDNUM_1000000       (1)   ///< 1,000,000 bps at 0.0% tolerance
#define DXL_BAUDNUM_500000        (3)   ///<   500,000 bps at 0.0% tolerance
#define DXL_BAUDNUM_400000        (4)   ///<   500,000 bps at 0.0% tolerance
#define DXL_BAUDNUM_250000        (7)   ///<   250,000 bps at 0.0% tolerance
#define DXL_BAUDNUM_200000        (9)   ///<   200,000 bps at 0.0% tolerance
#define DXL_BAUDNUM_115200        (16)  ///<   115,200 bps at -2.124% tolerance
#define DXL_BAUDNUM_57600         (34)  ///<    57,600 bps at  0.794 % tolerance
#define DXL_BAUDNUM_19200         (103) ///<    19,200 bps at -0.160% tolerance
#define DXL_BAUDNUM_9600          (207) ///<     9,600 bps at -0.160% tolerance

  // 
  // Extended baud number enumeration.
  //
#define DXL_BAUDNUM_EXT_2250000   (250) ///< 2,250,000 bps at 0.0% tolerance
#define DXL_BAUDNUM_EXT_2500000   (251) ///< 2,500,000 bps at 0.0% tolerance
#define DXL_BAUDNUM_EXT_3000000   (252) ///< 3,000,000 bps at 0.0% tolerance

  // Dynamixel default rate.
#define DXL_DEFAULT_BAUDNUMBER    (34)  ///< default baud number

  //
  // Packet field indices.
  //
#define DXL_ID          (2)   ///< servo id field packet index
#define DXL_LENGTH      (3)   ///< packet length field packet index
#define DXL_INSTRUCTION (4)   ///< instruction field packet index
#define DXL_ERRBIT      (4)   ///< error bits field packet index
#define DXL_PARAMETER   (5)   ///< start of parameter fields packet index

  //
  // Maximum transmit/receive lengths.
  //
#define DXL_MAXNUM_TXPARAM    (150)   ///< maximum number of tx parameters
#define DXL_MAXNUM_RXPARAM    (60)    ///< maximum number of rx parameters

  //
  // Instructions (commands).
  //
#define DXL_INST_PING         (1)   ///< ping servo
#define DXL_INST_READ         (2)   ///< read servo control table data
#define DXL_INST_WRITE        (3)   ///< write servo control table data
#define DXL_INST_REG_WRITE    (4)   ///< register write
#define DXL_INST_ACTION       (5)   ///< action
#define DXL_INST_RESET        (6)   ///< reset servo defaults
#define DXL_INST_SYNC_WRITE   (131) ///< synchronous write servo(s) data

  //
  // Servo Ids
  //
#define DXL_MIN_ID            (0)     ///< minimum servo id
#define DXL_MAX_ID            (253)   ///< maximum servo id
#define DXL_BROADCAST_ID      (254)   ///< broadcast "servo" id

  //
  // Servo error bits.
  //
#define DXL_ERRBIT_VOLTAGE      (1)   ///< voltage out-of-range error
#define DXL_ERRBIT_ANGLE        (2)   ///< servo angle out-of-range error
#define DXL_ERRBIT_OVERHEAT     (4)   ///< temperature out-of-range error
#define DXL_ERRBIT_RANGE        (8)   ///< parameter? out-of-range error
#define DXL_ERRBIT_CHECKSUM     (16)  ///< bad checksum error
#define DXL_ERRBIT_OVERLOAD     (32)  ///< load out-of-range error
#define DXL_ERRBIT_INSTRUCTION  (64)  ///< bad instruction error

  //
  // Communication result codes.
  //
#define DXL_COMM_TXSUCCESS    (0) ///< transmit success
#define DXL_COMM_RXSUCCESS    (1) ///< receive success
#define DXL_COMM_TXFAIL       (2) ///< transmit failure error
#define DXL_COMM_RXFAIL       (3) ///< receive failure error
#define DXL_COMM_TXERROR      (4) ///< packed transmit packet format error
#define DXL_COMM_RXWAITING    (5) ///< waiting for complete receive packet
#define DXL_COMM_RXTIMEOUT    (6) ///< receive timeout error
#define DXL_COMM_RXCORRUPT    (7) ///< receive corrupted packet

  /*!
   * \brief half-duplex control transmit callback type.
   *
   * \param arg Callback user argument.
   */
  typedef void (*dxl_hdctl_tx_cb_t)(void *arg);

  /*!
   * \brief half-duplex control receive callback type.
   *
   * \param arg Callback user argument.
   * \param arg Transmit packet length.
   */
  typedef void (*dxl_hdctl_rx_cb_t)(void *arg, size_t txlen);

  //
  // Foward declarations.
  //
  class dxlhal;

  //----------------------------------------------------------------------------
  // Class dxl
  //----------------------------------------------------------------------------
 
  class dxl
  {
  public:
    /*!
     * \brief Default constructor.
     */
    dxl();

    /*!
     * \brief Destructor.
     */
    ~dxl();

    /*!
     * \brief Open Dynamixel Bus serial interface.
     *
     * The serial device is opened and attributes set.
     *
     * \param deviceName    Serial device name.
     * \param baudrate      Dynamixel baud rate.
     *
     * \return Returns 1 on success, 0 on failure.
     */
    int open(const char *deviceName, int baudrate);
    
    /*!
     * \brief Initialize Dynamixel Bus USB serial interface by index.
     *
     * The serial device is opened and attributes set.
     *
     * \note This method is deprecated. The preferred method is open().
     *
     * \param deviceIndex   Device number of /dev/ttyUSBn, n == deviceIndex.
     * \param baudnum       Dynamixel baud number.
     *
     * \return Returns 1 on success, 0 on failure.
     */
    int dxl_initialize(int deviceIndex, int baudnum);

    /*!
     * \brief Close the Dynamixel Bus serial interface.
     *
     * The serial device is closed.
     */
    void close();
    
    /*!
     * \brief Set Dynamixel Bus serial device baud rate.
     *
     * \param baudrate  Baud rate.
     *
     * \return Returns 1 on success, 0 on failure.
     */
    int setBaudRate(int baudrate);

    /*!
     * \brief Set serial half-duplex callbacks.
     *
     * The relavent callback is made just prior to transmitting or receiving a
     * packet.
     *
     * \param enable_tx_cb  Enable transmit callback.
     * \param enable_rx_cb  Enable receive callback.
     * \param arg           Tx/Rx callback parameter.
     *
     * \return Returns 1 on success, 0 on failure.
     */
    int setHalfDuplexCallbacks(dxl_hdctl_tx_cb_t  enable_tx_cb,
                               dxl_hdctl_rx_cb_t  enable_rx_cb,
                               void              *arg);
    
    /*!
     * \brief Get the Dynamixel Bus serial device file descriptor.
     *
     * \return Returns \>= 0 if the device is open, -1 otherwise.
     */
    int getFd();
    
    /*!
     * \brief Set transmit packet servo id.
     *
     * \param id  Servo id or broadcast id.
     */
    void setTxPacketId(int id);
    
    /*!
     * \brief Set transmit packet instruction.
     *
     * \param instruction Request instruction id.
     */
    void setTxPacketInstruction(int instruction);
    
    /*!
     * \brief Set transmit packet parameter value
     *
     * \param index   Zero offset parameter index.
     * \param value   Parameter byte value.
     */
    void setTxPacketParameter(int index, int value);
    
    /*!
     * \brief Set transmit packet length.
     *
     * \param length    Length in bytes.
     */
    void setTxPacketLength(int length);
    
    /*!
     * \brief Get receive packet error bits field.
     *
     * \return Error bits.
     */
    unsigned int getRxPacketErrBits();
    
    /*!
     * \brief Test if error bit is set on received packet.
     *
     * \param errbit  Error bit to test.
     *
     * \return Returns true or false.
     */
    bool getRxPacketError(unsigned int errbit);
    
    
    /*!
     * \brief Get receive packet length field.
     *
     * \return Receive packet expected length.
     */
    int getRxPacketLength();
    
    /*!
     * \brief Get receive packet parameter value
     *
     * \param index   Zero offset parameter index.
     *
     * \return Paramter byte value.
     */
    int getRxPacketParameter(int index);
    
    /*!
     * \brief Make 16-bit word.
     *
     * \param lowbyte   Least significant byte.
     * \param highbyte  Most significant byte.
     *
     * \return Packed word.
     */
    int makeWord(int lowbyte, int highbyte);
    
    /*!
     * \brief Get low byte from word.
     *
     * \param word  16-bit word.
     *
     * \return  Unpacked least significant byte.
     */
    int getLowByte(int word);
    
    /*!
     * \brief Get high byte from word.
     *
     * \param word  16-bit word.
     *
     * \return  Unpacked most significant byte.
     */
    int getHighByte(int word);
    
    /*!
     * \brief Transmit a previously packed instruction packet.
     *
     * \par Transmit Packet
     * 0xff 0xff ID LENGTH INSTRUCTION paramters CHECKSUM
     */
    void txPacket();
    
    /*!
     * \brief Receive status packet.
     *
     * \par Non-Broadcast Receive Packet
     * 0xff 0xff ID LENGTH ERRBIT paramters CHECKSUM
     */
    void rxPacket();
    
    /*!
     * \brief Transmit instruction packet and receive status response packet.
     *
     * Use dxl_get_result() to check for success or failure.
     *
     * Use the tranmit packing functions to build the instruction packet and the
     * receive unpacking function to get any read data.
     */
    void txrxPacket();
    
    /*!
     * \brief Get last transmit/receive result.
     *
     * \return Result code.
     */
    int getResult();
    
    /*!
     * \brief Ping servo.
     *
     * Use dxl_get_result() to check for success or failure.
     *
     * \param id    Servo id.
     */
    void ping(int id);
    
    /*!
     * \brief Read byte from servo's control table.
     *
     * Use dxl_get_result() to check for success or failure.
     *
     * \param id      Servo id.
     * \param address Control table address.
     *
     * \return Read 8-bit byte.
     */
    int readByte(int id, int address);
    
    /*!
     * \brief Write byte to servo's control table.
     *
     * Use dxl_get_result() to check for success or failure.
     *
     * \param id      Servo id.
     * \param address Control table address.
     * \param value   8-bit value to write.
     */
    void writeByte(int id, int address, int value);
    
    /*!
     * \brief Read word from servo's control table.
     *
     * Use dxl_get_result() to check for success or failure.
     *
     * \param id      Servo id.
     * \param address Control table address.
     *
     * \return Read 16-bit word.
     */
    int readWord(int id, int address);
    
    /*!
     * \brief Write word to servo's control table.
     *
     * Use dxl_get_result() to check for success or failure.
     *
     * \param id      Servo id.
     * \param address Control table address.
     * \param value   16-bit value to write.
     */
    void writeWord(int id, int address, int value);

  protected:
    // packets
    uint8_t m_bufInstructionPacket[DXL_MAXNUM_TXPARAM+10];
                              ///< instruction transmit packet buffer
    uint8_t m_bufStatusPacket[DXL_MAXNUM_RXPARAM+10];
                              ///< status receive packet buffer

    // communication
    int   m_nRxPacketLength;  ///< expected received packet length
    int   m_nRxGetLength;     ///< current received packet length
    int   m_nCommStatus;      ///< communication status
    bool  m_bBusUsing;        ///< bus is [not] busy

    // half-duplex control callbacks.
    dxl_hdctl_tx_cb_t  m_fnEnableTx;  ///< enable tx callback
    dxl_hdctl_rx_cb_t  m_fnEnableRx;  ///< enable rx callback
    void              *m_pTxRxArg;    ///< tx/rx argument passback

    // implementaion hardware abstraction layer
    dxlhal            *m_pDxlHal;

    /*!
     * \brief Convert Dynamixel baud number to baud rate.
     *
     * \note This method is deprecated. The preferred method is open().
     *
     * \param baudnum   Dynamixel baud number.
     * 
     * \return Baud rate.
     */
    float dxl_baudnum_to_baudrate(int baudnum);
  };

} // namespace libdxl


#endif // _DXL_H
