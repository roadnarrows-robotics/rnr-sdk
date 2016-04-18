////////////////////////////////////////////////////////////////////////////////
//
// Package:   Laelaps
//
// File:      laeImu.h
//
/*! \file
 *
 * $LastChangedDate: 2016-02-09 15:08:50 -0700 (Tue, 09 Feb 2016) $
 * $Rev: 4308 $
 *
 * \brief Laelaps built-in Inertial Measurement Unit class interface.
 *
 * The current Laelaps uses the open-source CleanFlight firmware loaded
 * on a Naze32 controller. The interface is serial USB.
 *
 * \sa https://github.com/cleanflight
 *
 * \author Robin Knight (robin.knight@roadnarrows.com)
 *
 * \par Copyright:
 * (C) 2015-2016  RoadNarrows
 * (http://www.RoadNarrows.com)
 * \n All Rights Reserved
 */
/*
 * @EulaBegin@
 * 
 * Unless otherwise stated explicitly, all materials contained are copyrighted
 * and may not be used without RoadNarrows LLC's written consent,
 * except as provided in these terms and conditions or in the copyright
 * notice (documents and software) or other proprietary notice provided with
 * the relevant materials.
 * 
 * IN NO EVENT SHALL THE AUTHOR, ROADNARROWS LLC, OR ANY 
 * MEMBERS/EMPLOYEES/CONTRACTORS OF ROADNARROWS OR DISTRIBUTORS OF THIS SOFTWARE
 * BE LIABLE TO ANY PARTY FOR DIRECT, INDIRECT, SPECIAL, INCIDENTAL, OR
 * CONSEQUENTIAL DAMAGES ARISING OUT OF THE USE OF THIS SOFTWARE AND ITS
 * DOCUMENTATION, EVEN IF THE AUTHORS OR ANY OF THE ABOVE PARTIES HAVE BEEN
 * ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 * 
 * THE AUTHORS AND  ROADNARROWS LLC SPECIFICALLY DISCLAIM ANY WARRANTIES,
 * INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND
 * FITNESS FOR A PARTICULAR PURPOSE. THE SOFTWARE PROVIDED HEREUNDER IS ON AN
 * "AS IS" BASIS, AND THE AUTHORS AND DISTRIBUTORS HAVE NO OBLIGATION TO
 * PROVIDE MAINTENANCE, SUPPORT, UPDATES, ENHANCEMENTS, OR MODIFICATIONS.
 * 
 * @EulaEnd@
 */
////////////////////////////////////////////////////////////////////////////////

#ifndef _LAE_IMU_H
#define _LAE_IMU_H

#include <pthread.h>

#ifndef SWIG
#include <string>
#endif // SWIG

#include "rnr/rnrconfig.h"

#include  "Laelaps/laelaps.h"


#ifdef SWIG
// const --> %constant in swig interface .i file.
#define STATIC_STR const char*
#else
#define STATIC_STR static const char* const
#endif // SWIG

/*!
 *  \brief The sensor namespace.
 */
#ifndef SWIG
namespace sensor
{
  // the IMU namespace
  namespace imu
  {
#endif // SWIG
    //--------------------------------------------------------------------------
    // IMU Command Data
    //--------------------------------------------------------------------------

    /*!
     * \brief Axes indices.
     */
    enum Axis
    {
      X = 0,    ///< x axis index
      Y = 1,    ///< y axis index
      Z = 2     ///< z axis index
    };
  
    const int NumOfAxes = 3;  ///< maximum number of axes per sensor component.

    /*!
     * \brief Roll, pitch, and yaw indices.
     */
    enum RPY
    {
      ROLL  = 0,  ///< roll index
      PITCH = 1,  ///< pitch index  
      YAW   = 2   ///< yaw index
    };
  

#ifndef SWIG
    //--------------------------------------------------------------------------
    // LaeImu Virtual Base Class
    //--------------------------------------------------------------------------
    
    /*!
     * Inertia Measurement Unit virtual base class with serial interface.
     */
    class LaeImu
    {
    public:
      /*!
       * \brief Default constructor.
       *
       * \param strIdent    IMU identity string.
       */
      LaeImu(std::string strIdent="Virtual IMU");
  
      /*!
       * \brief Destructor.
       */
      virtual ~LaeImu();
  
      /*!
       * \bried Open connection to IMU.
       *
       * \param strDevName  Serial device name.
       * \param nBaudRate   Serial device baud rate.
       *
       * \copydoc doc_return_std
       */
      virtual int open(const std::string &strDevName, int nBaudRate);
  
      /*!
       * \brief Close connection to motor controller.
       *
       * \copydoc doc_return_std
       */
      virtual int close();
  
      /*!
       * \brief Check if IMU serial interface is open.
       *
       * \return Returns true or false.
       */
      virtual bool isOpen()
      {
        return m_fd >= 0? true: false;
      }

      /*!
       * \brief Get the total IMU degress of freedom.
       *
       * \return DoF
       */
      virtual int numDoFs()
      {
        return 0;
      }

      /*!
       * \brief Test if IMU has an accelerometer.
       *
       * \return Returns true or false.
       */
      virtual bool hasAccelerometer()
      {
        return false;
      }

      /*!
       * \brief Test if IMU has an gyroscope.
       *
       * \return Returns true or false.
       */
      virtual bool hasGyroscope()
      {
        return false;
      }

      /*!
       * \brief Test if IMU has an magnetometer.
       *
       * \return Returns true or false.
       */
      virtual bool hasMagnetometer()
      {
        return false;
      }

      /*!
       * \brief Configure sensor and class instance data.
       *
       * Call after connection is opened.
       *
       * \copydoc doc_return_std
       */
      virtual int configure() = 0;
  
      /*!
       * \brief Read sensor identity values.
       *
       * \param [out] strIdent    Hardware specific, formated identity string.
       *
       * \copydoc doc_return_std
       */
      virtual int readIdentity(std::string &strIdent) = 0;
  
      /*!
       * \brief Read sensor raw IMU values.
       * 
       * Depending on the sensor and the board, raw values read:
       *  \li accelerometer
       *  \li gyroscope
       *  \li magnetometer
       *  \li GPS
       *  \li barometer
       *  \li temperature
       *  \li accurate timing
       *  \li attitude
       *  \li quaternions
       *
       * \copydoc doc_return_std
       */
      virtual int readRawImu() = 0;
  
      /*!
       * \brief Convert last read IMU values to International System of Units.
       *
       * \copydoc doc_return_std
       */
      virtual int convertRawToSI() = 0;
  
      /*!
       * \brief Compute all IMU values. 
       *
       * \todo
       */
      virtual void compute();

      /*!
       * \brief Compute the quaternion from the IMU data.
       *
       * \todo
       */
      virtual void computeQuaternion();

      /*!
       * \brief Compute the velocity, position, and any other dynamics from the
       * IMU data.
       *
       * \todo
       */
      virtual void computeDynamics();

      /*!
       * \brief Get IMU identity.
       *
       * \return String.
       */
      std::string getIdentity()
      {
        return m_strIdent;
      }
      /*!
       * \brief Get IMU device name.
       *
       * \return String.
       */
      std::string getDevName()
      {
        return m_strDevName;
      }

      /*!
       * \brief Get the last read raw inertia data.
       *
       * Not applicable data are set to zero.
       *
       * \param [out] accel   Raw accelerometer data.
       *                      The array size must be \h_ge \ref NumOfAxes.
       * \param [out] gyro    Raw gyroscope data.
       *                      The array size must be \h_ge \ref NumOfAxes.
       */
      virtual void getRawInertiaData(int accel[], int gyro[]);

      /*!
       * \brief Get the last read and converted inertia data.
       *
       * Not applicable data are set to zero.
       *
       * \param [out] accel   Accelerometer data (m/s^2).
       *                      The array size must be \h_ge \ref NumOfAxes.
       * \param [out] gyro    Gyroscope data (radians/s).
       *                      The array size must be \h_ge \ref NumOfAxes.
       */
      virtual void getInertiaData(double accel[], double gyro[]);

      /*!
       * \brief Get the last read IMU (vehicle) attitude.
       *
       * Data are set to zero if not availble.
       *
       * \param [out] roll    Vehicle roll (radians).
       * \param [out] pitch   Vehicle pitch (radians).
       * \param [out] yaw     Vehicle yaw (radians).
       */
      virtual void getAttitude(double &roll, double &pitch, double &yaw);

      // FUTURE
      //virtual void getRawMagData(int mag[]);
      //virtual void getMagData(double mag[]);
      //virtual void getRawBarometerData(int &baro);
      //virtual void getBarometerData(double &baro);
      //virtual void getRawTemperatureData(int &temp);
      //virtual void getGps(GPS &gps);
      //virtual void getTemperatureData(double &temp);
      //virutal void getQuaternion(Quaternian &q);
      //virtual void getDynamics(double &vel[], double &pos[]);
      // FUTURE

    protected:
      //
      // Hardware interface.
      //
      std::string   m_strIdent;     ///< IMU identity
      std::string   m_strDevName;   ///< serial device name
      int           m_nBaudRate;    ///< device baudrate
      int           m_fd;           ///< opened device file descriptor

      //
      // Raw sensor values.
      //
      int     m_accelRaw[NumOfAxes];    ///< accelerometer raw values
      int     m_gyroRaw[NumOfAxes];     ///< gyroscope raw values
      int     m_magRaw[NumOfAxes];      ///< magnetometer raw values
      int     m_rpyRaw[NumOfAxes];      ///< roll,pitch,yaw raw values

      //
      // Converted sensor values in SI units.
      //
      double  m_accel[NumOfAxes];       ///< accelerometer (m/s^2)
      double  m_gyro[NumOfAxes];        ///< gyrscope (radians/s)
      double  m_mag[NumOfAxes];         ///< magnetometer (tesla)
      double  m_rpy[NumOfAxes];         ///< roll,pitch,yaw (radians)

      // FUTURE
      // Quaternion m_quaternion;
      // double m_vel[];
      // double m_pos[];
      // int m_baroRaw;
      // double m_baro;
      // int m_tempRaw;
      // double m_temp;
      // GPS m_gps;
      // FUTURE

      // mutual exclusion
      pthread_mutex_t m_mutex;          ///< mutex
  
      /*!
       * \brief Lock the shared resource.
       *
       * The lock()/unlock() primitives provide for safe multi-threading.
       *
       * \par Context:
       * Any.
       */
      void lock()
      {
        pthread_mutex_lock(&m_mutex);
      }
    
      /*!
       * \brief Unlock the shared resource.
       *
       * \par Context:
       * Any.
       */
      void unlock()
      {
        pthread_mutex_unlock(&m_mutex);
      }

      /*!
       * \brief Zero IMU data.
       */
      void zeroData();
    };
#endif // SWIG


    //--------------------------------------------------------------------------
    // Multiwii Serial Protocol Interface
    //--------------------------------------------------------------------------

    /*!
     * \brief Multiwii Serial Protocal Interface
     *
     * Several IMU boards use the MSP interface.
     *
     * Command:  <preamble>MspDirTo<size><cmdid>[<data>]<crc>
     *
     * Response: <preamble>MspDirFrom<size><cmdid>[<data>]<crc>
     */
#ifndef SWIG
    namespace msp
    {
#endif // SWIG
      //
      // Message preambles
      //
      STATIC_STR MspPreamble    = "$M";   ///< message preamble
      STATIC_STR MspDirTo       = "<";    ///< to imu
      STATIC_STR MspDirFrom     = ">";    ///< from imu
      STATIC_STR MspCmdPreamble = "$M<";  ///< command preamble
      STATIC_STR MspRspPreamble = "$M>";  ///< response preamble

      //
      // Field positions
      //
      static const int MspFieldPosDir       = 3;  ///< dir field position
      static const int MspFieldPosSize      = 3;  ///< data size field position
      static const int MspFieldPosCmdId     = 4;  ///< command id field position
      static const int MspFieldPosDataStart = 5;  ///< data start field position

      //
      // Message lengths
      //
      static const int MspCmdHdrLen = 5;  ///< command header len
      static const int MspCmdMinLen = 6;  ///< MspCmdHdrLen+1 command min len
      static const int MspCmdMaxLen = 32; ///< response max len
      static const int MspRspHdrLen = 5;  ///< response header len
      static const int MspRspMinLen = 6;  ///< MspRspHdrLen+1 response min len
      static const int MspRspMaxLen = 32; ///< response max len

      //
      // Command Ids
      //
      static const uint_t MspCmdIdIdent    = 100; ///< read board/sensor idents
      static const uint_t MspCmdIdRawImu   = 102; ///< read raw imu data
      static const uint_t MspCmdIdAttitude = 108; ///< read raw attitude data

      /*!
       * \brief Board identity structure.
       */
      struct MspIdent
      {
        uint_t  m_uFwVersion;   ///< firmware version
        uint_t  m_uMultiType;   ///< multiwii type
        uint_t  m_uMspVersion;  ///< MSP version
        uint_t  m_uCaps;        ///< capabilities or'd bits
      };

      //
      // Conversion Factors
      //
      static const double MspGToMPerSec2            = 9.80665;
                            ///< standard g in m/s^2
      static const double MspMpu6050RawToG          = 1.0 / 512.0;
                            ///< MPU-6050 acceleration raw value to g's
      static const double MspMpu6050RawToDegPerSec  = 1.0 / 4.096;
                            ///< MPU-6050 rotation raw value to degrees/second
      static const double MspAttitudeRawToDeg       = 0.1;
                            ///< board computed attitude raw value to degreess

#ifndef SWIG
    } // namespace msp
#endif // SWIG

#ifndef SWIG
    //--------------------------------------------------------------------------
    // LaeImuCleanFlight Class
    //--------------------------------------------------------------------------
    
    /*!
     * Inertia Measurement Unit class running the CleanFlight firmware.
     */
    class LaeImuCleanFlight : public LaeImu
    {
    public:
      //
      // Serial baud rates
      //
      static const int SerBaud9600    =    9600;  ///< 9600 serial baudrate 
      static const int SerBaud19200   =   19200;  ///< 19200 serial baudrate 
      static const int SerBaud38400   =   38400;  ///< 38400 serial baudrate 
      static const int SerBaud57600   =   38400;  ///< 38400 serial baudrate 
      static const int SerBaud115200  =  115200;  ///< 115200 serial baudrate

      static const int SerBaudDft = SerBaud115200;  ///< default serial baudrate
  
      //
      // Response timeout
      //
      static const uint_t TCmdTimeout  = 1000;  ///< command timeout (usec)
      static const uint_t TRspTimeout  = 12000; ///< response timeout (usec)
      static const uint_t TFlushDelay  = 100;   ///< flush buffer delay (usec)

      /*!
       * \brief Default constructor.
       */
      LaeImuCleanFlight();
  
      /*!
       * \brief Destructor.
       */
      virtual ~LaeImuCleanFlight();
  
      /*!
       * \brief Get the total degress of freedom of the IMU sensor.
       *
       * \return DoF
       */
      virtual int numDoFs()
      {
        return 6;
      }

      /*!
       * \brief Test if IMU has an accelerometer.
       *
       * \return Returns true or false.
       */
      virtual bool hasAccelerometer()
      {
        return true;
      }

      /*!
       * \brief Test if IMU has an gyroscope.
       *
       * \return Returns true or false.
       */
      virtual bool hasGyroscope()
      {
        return true;
      }

      /*!
       * \brief Configure sensor and class instance data.
       *
       * Call after connection is opened.
       *
       * The CleanFlight defaults are acceptable. No configuration is
       * necessary todate.
       *
       * \copydoc doc_return_std
       */
      virtual int configure();
  
      /*!
       * \brief Reload with new configuration.
       *
       * The CleanFlight defaults are acceptable. No re-configuration is
       * necessary todate.
       *
       * \copydoc doc_return_std
       */
      virtual int reload();
  
      /*!
       * \brief Read sensor identity values.
       *
       * \param [out] strIdent    Hardware specific, formated identity string.
       *
       * \copydoc doc_return_std
       */
      virtual int readIdentity(std::string &strIdent);
  
      /*!
       * \brief Read sensor and board raw IMU values.
       * 
       * The CleanFlight Naze32 with MPU-6050 sensor has:
       *  \li accelerometer
       *  \li gyroscope
       *  \li attitude
       *
       * \copydoc doc_return_std
       */
      virtual int readRawImu();
  
      /*!
       * \brief Read sensors inertia data.
       * 
       * \copydoc doc_return_std
       */
      virtual int readRawInertia();
  
      /*!
       * \brief Read board's calculated roll, pitch, and yaw raw values.
       * 
       * \copydoc doc_return_std
       */
      virtual int readRawRollPitchYaw();
  
      /*!
       * \brief Convert last read IMU values to International System of Units.
       *
       * \copydoc doc_return_std
      */
      virtual int convertRawToSI();

    protected:
      /*!
       * \brief Read CleanFlight board's identity values.
       *
       * MSP command/response.
       *
       * \param [out] ident Identity.
       *
       * \copydoc doc_return_std
       */
      int mspReadIdent(msp::MspIdent &ident);

      /*!
       * \brief Read raw IMU data.
       *
       * Read data are updated to local class object member data.
       *
       * MSP command/response.
       *
       * \copydoc doc_return_std
       */
      int mspReadRawImu();

      /*!
       * \brief
       *
       * MSP command/response.
       *
       * \param
       *
       * \copydoc doc_return_std
       */
      int mspReadAttitude();

      /*!
       * \brief Send command to IMU.
       *
       * \param cmdId         MSP command id.
       * \param [in] cmdData  Optional command data.
       * \param lenData       Length of command data. Zero if no data.
       *
       * \copydoc doc_return_std
       */
      int sendCmd(uint_t cmdId, byte_t cmdData[], size_t lenData);

      /*!
       * \brief
       *
       * \param cmdId         Response's associated MSP command id.
       * \param [out] rspData Optional response data.
       * \param lenData       Length of response data. Zero if no data.
       *
       * \copydoc doc_return_std
       */
      int receiveRsp(uint_t cmdId, byte_t rspData[], size_t lenData);

      /*!
       * \brief Attempt to resynchronize the serial communication between the
       * host and the IMU.
       */
      void resyncComm();

      /*!
       * \brief Flush serial input and output FIFOs, discarding all data.
       *
       * \param t   Delay time (usec) before flushing. Zero means no delay.
       */
      void flush(uint_t t);

      /*!
       * \brief Pack 16-bit signed value into buffer.
       *
       * Order is little-endian (LSB first).
       *
       * \param [in] val   Value to pack.
       * \param [out] buf  Destination buffer.
       *
       * \return Number of bytes packed(2).
       */
      int pack16(int val, byte_t buf[])
      {
        return pack16((uint_t &)val, buf);
      }

      /*!
       * \brief Pack 16-bit unsigned value into buffer.
       *
       * Order is little-endian (LSB first).
       *
       * \param [in] val   Value to pack.
       * \param [out] buf  Destination buffer.
       *
       * \return Number of bytes packed(2).
       */
      int pack16(uint_t val, byte_t buf[]);

      /*!
       * \brief Unpack 16-bit signed value from buffer.
       *
       * Buffer is little-endian (LSB first).
       *
       * \param [in] buf  Source buffer.
       * \param [out] val Unpacked value.
       *
       * \return Number of bytes unpacked(2).
       */
      int unpack16(byte_t buf[], int &val);

      /*!
       * \brief Unpack 16-bit unsigned value from buffer.
       *
       * Buffer is little-endian (LSB first).
       *
       * \param [in] buf  Source buffer.
       * \param [out] val Unpacked value.
       *
       * \return Number of bytes unpacked(2).
       */
      int unpack16(byte_t buf[], uint_t &val);

      /*!
       * \brief Pack 32-bit signed value into buffer.
       *
       * Order is little-endian (LSB first).
       *
       * \param [in] val   Value to pack.
       * \param [out] buf  Destination buffer.
       *
       * \return Number of bytes packed(4).
       */
      int pack32(int val, byte_t buf[])
      {
        return pack32((uint_t)val, buf);
      }

      /*!
       * \brief Pack 32-bit unsigned value into buffer.
       *
       * Order is little-endian (LSB first).
       *
       * \param [in] val   Value to pack.
       * \param [out] buf  Destination buffer.
       *
       * \return Number of bytes packed(4).
       */
      int pack32(uint_t val, byte_t buf[]);

      /*!
       * \brief Unpack 32-bit signed value from buffer.
       *
       * Buffer is little-endian (LSB first).
       *
       * \param [in] buf  Source buffer.
       * \param [out] val Unpacked value.
       *
       * \return Number of bytes unpacked(4).
       */
      int unpack32(byte_t buf[], int &val);

      /*!
       * \brief Unpack 32-bit unsigned value from buffer.
       *
       * Buffer is little-endian (LSB first).
       *
       * \param [in] buf  Source buffer.
       * \param [out] val Unpacked value.
       *
       * \return Number of bytes unpacked(4).
       */
      int unpack32(byte_t buf[], uint_t &val);

    };
#endif // SWIG

#ifndef SWIG
  } // namespace imu

} // namespace sensor
#endif // SWIG


#endif // _LAE_IMU_H
