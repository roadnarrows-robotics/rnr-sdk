////////////////////////////////////////////////////////////////////////////////
//
// Package:   Eudoxus
//
// Library:   libEudoxus
//
// File:      euPcd.cxx
//
/*! \file
 *
 * $LastChangedDate: 2016-01-18 14:13:40 -0700 (Mon, 18 Jan 2016) $
 * $Rev: 4263 $
 *
 * \brief Eudoxus Point Cloud Data definitions and helper functions.
 *
 * \author Robin Knight (robin.knight@roadnarrows.com)
 *
 * \copyright
 *   \h_copy 2012-2017. RoadNarrows LLC.\n
 *   http://www.roadnarrows.com\n
 *   All Rights Reserved
 */
/*
 * @EulaBegin@
// Unless otherwise noted, all materials contained are copyrighted and may not
// be used except as provided in these terms and conditions or in the copyright
// notice (documents and software ) or other proprietary notice provided with
// the relevant materials.
//
//
// IN NO EVENT SHALL THE AUTHOR, ROADNARROWS, OR ANY MEMBERS/EMPLOYEES/
// CONTRACTORS OF ROADNARROWS OR DISTRIBUTORS OF THIS SOFTWARE BE LIABLE TO ANY
// PARTY FOR DIRECT, INDIRECT, SPECIAL, INCIDENTAL, OR CONSEQUENTIAL
// DAMAGES ARISING OUT OF THE USE OF THIS SOFTWARE AND ITS DOCUMENTATION,
// EVEN IF THE AUTHORS OR ANY OF THE ABOVE PARTIES HAVE BEEN ADVISED OF
// THE POSSIBILITY OF SUCH DAMAGE.
//
// THE AUTHORS AND  ROADNARROWS SPECIFICALLY DISCLAIM ANY WARRANTIES,
// INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND
// FITNESS FOR A PARTICULAR PURPOSE. THE SOFTWARE PROVIDED HEREUNDER IS ON AN
// "AS IS" BASIS, AND THE AUTHORS AND DISTRIBUTORS HAVE NO OBLIGATION TO
// PROVIDE MAINTENANCE, SUPPORT, UPDATES, ENHANCEMENTS, OR MODIFICATIONS.
//
 * @EulaEnd@
 */
////////////////////////////////////////////////////////////////////////////////

#include "Eudoxus/euConf.h"

#include <sys/time.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>

#include <string>
#include <cfloat>

#include "rnr/rnrconfig.h"
#include "rnr/log.h"

#ifdef EU_HAS_PCL
#include <Eigen/Geometry>

#include <pcl/common/common_headers.h>
#include <pcl/io/pcd_io.h>

#endif // EU_HAS_PCL

#include "Eudoxus/Eudoxus.h"
#include "Eudoxus/euHeader.h"
#include "Eudoxus/euIo.h"
#include "Eudoxus/euPcd.h"

using namespace std;
using namespace eu;

// ---------------------------------------------------------------------------
// EuPcdHeader Base Class
// ---------------------------------------------------------------------------

/*!
 * \brief Default constructor.
 */
EuPcdHeader::EuPcdHeader(EuHdrType   eHdrType,
                         string      strHdrName,
                         const char *sHdrFmt,
                         string      strDataFmt)
 :
    EuHeader(eHdrType, strHdrName),
    m_sHdrFmt(sHdrFmt),
    m_strDataFmt(strDataFmt)
{
  m_resolution.m_uWidth   = 0;
  m_resolution.m_uHeight  = 0;
#ifdef EU_HAS_PCL
  m_origin                = Eigen::Vector4f::Zero();
  m_orientation           = Eigen::Quaternionf::Identity();
#endif // EU_HAS_PCL
  m_bufHdr[0]             = 0;
}

/*!
 * \brief Destructor.
 */
EuPcdHeader::~EuPcdHeader()
{
}

ssize_t EuPcdHeader::pack(byte_t buf[], size_t sizeBuf)
{
  int         n;

  m_uHdrSize    = 0;
  m_uTotalSize  = 0;

  n = snprintf((char *)buf, sizeBuf, m_sHdrFmt,
      // WIDTH
      m_resolution.m_uWidth,
      // HEIGHT
      m_resolution.m_uHeight,
#ifdef EU_HAS_PCL
      // VIEWPOINT (origin: tx ty tz)
      m_origin[0], m_origin[1], m_origin[2],
      // VIEWPOINT (orientation: qw qx qy qz)
      m_orientation.w(), m_orientation.x(), m_orientation.y(),
      m_orientation.z(),
#endif // EU_HAS_PCL
      // POINTS
      m_uRecordCnt,
      // DATA
      m_strDataFmt.c_str());

  if( n < 0 )
  {
    LOGERROR("Invalid PCD header value(s).");
    buf[0] = 0;
    return -EU_ECODE_BAD_VAL;
  }

  // buffer too small so truncate
  else if( n >= sizeBuf )
  {
    LOGERROR("PCD header output buffer too small.");
    buf[0] = 0;
    return -EU_ECODE_TOO_SMALL;
  }

  m_uHdrSize    = n;
  m_uTotalSize  = m_uHdrSize + m_uDataSize;

  return (ssize_t)n;
}

ssize_t EuPcdHeader::unpack(const byte_t buf[], size_t lenBuf)
{
  LOGERROR("PCD unpack routine unsupported. Use EuPcdIo.");
  return -EU_ECODE_NO_EXEC;
}


#ifdef EU_OLD_1_6
#ifdef EU_HAS_PCL

// ---------------------------------------------------------------------------
// EuPcdIo Class
// ---------------------------------------------------------------------------

ssize_t EuPcdIo::readbuf(byte_t buf[],
                         size_t sizeBuf,
                         //pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud)
                         pcl::PCLPointCloud2 &cloud)
{
  Eigen::Vector4f origin;
  Eigen::Quaternionf orientation; 
  int version;

  return EuPcdIo::readbuf(buf, sizeBuf, cloud, origin, orientation, version);
}

ssize_t EuPcdIo::readbuf(byte_t buf[],
                         size_t sizeBuf,
                         //pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud,
                         pcl::PCLPointCloud2 &cloud,
                         Eigen::Vector4f &origin,
                         Eigen::Quaternionf &orientation, 
                         int &pcd_version)
{
  int       data_type;
  ssize_t   n;

  if( (n = EuPcdIo::readbufHeader(buf, sizeBuf, cloud, origin, orientation,
                             pcd_version, data_type)) <= 0 )
  {
    return -1;
  }

  switch( data_type )
  {
    case 0:   // ascii
      n = EuPcdIo::readbufDataAscii(buf+n, sizeBuf-n, cloud);
      break;
    case 1:   // binary
      n = EuPcdIo::readbufDataBinary(buf+n, sizeBuf-n, cloud);
      break;
    case 2:   // binary compressed
    default:
      LOGERROR("%d: Unsupported/unknown PCD data type.");
      return -1;
  }

  return n <= 0? -1: 0;
}

ssize_t EuPcdIo::readbufHeader(byte_t buf[],
                               size_t sizeBuf,
                               //pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud,
                         pcl::PCLPointCloud2 &cloud,
                               Eigen::Vector4f &origin,
                               Eigen::Quaternionf &orientation, 
                               int &pcd_version,
                               int &data_type)
{
  // Default values
  data_type   = 0;    // ascii
  pcd_version = pcl::PCDReader::PCD_V7;
  origin      = Eigen::Vector4f::Zero();
  orientation = Eigen::Quaternionf::Identity();
 
  // clear values
  cloud.clear();

  // By default, assume that there are _no_ invalid (e.g., NaN) points
  //cloud.is_dense = true;

  int         nr_points = 0;
  std::string line;
  size_t      sizeHdr = 0;

  int specified_channel_count = 0;

  if( sizeBuf == 0 )
  {
    LOGERROR("[pcl::PCDReader2::readHeader] Header length is zero.");
    return -1;
  }

  // field_sizes represents the size of one element in a field
  // (e.g., float = 4, char = 1)
  // field_counts represents the number of elements in a field
  // (e.g., x = 1, normal_x = 1, fpfh = 33)
  std::vector<int> field_sizes, field_counts;

  // field_types represents the type of data in a field
  // (e.g., F = float, U = unsigned)
  std::vector<char> field_types;
  std::vector<std::string> st;

  // Read the header and fill it in with wonderful values
  try
  {
    while( sizeHdr < sizeBuf )
    {
      sizeHdr += EuPcdIo::getline(buf+sizeHdr, sizeBuf-sizeHdr, line);

      // Ignore empty lines
      if( line == "" )
      {
        continue;
      }

      // Tokenize the line
      boost::trim(line);
      boost::split(st, line, boost::is_any_of("\t\r "),
                             boost::token_compress_on);

      std::stringstream sstream(line);
      sstream.imbue(std::locale::classic());

      std::string line_type;
      sstream >> line_type;

      // Ignore comments
      if( line_type.substr(0, 1) == "#" )
      {
        continue;
      }

      // Version numbers are not needed for now, but we are checking to see if
      // they're there
      else if( line_type.substr(0, 7) == "VERSION" )
      {
        // add version number parsing
        continue;
      }

      // Get the field indices (check for COLUMNS too for backwards
      // compatibility)
      else if( (line_type.substr(0, 6) == "FIELDS") ||
               (line_type.substr(0, 7) == "COLUMNS") )
      {
        specified_channel_count = static_cast<int>(st.size() - 1);

        // Allocate enough memory to accommodate all fields
        cloud->resize(specified_channel_count);
        for(int i = 0; i < specified_channel_count; ++i)
        {
          std::string col_type = st.at(i + 1);
          cloud->fields[i].name = col_type;
        }

        // Default the sizes and the types of each field to float32 to avoid
        // crashes while using older PCD files
        int offset = 0;
        for(int i = 0; i < specified_channel_count; ++i, offset += 4)
        {
          cloud->fields[i].offset   = offset;
          cloud->fields[i].datatype = pcl::PCLPointField::FLOAT32;
          cloud->fields[i].count    = 1;
        }
        cloud.point_step = offset;
      }

      // Get the field sizes
      else if( line_type.substr(0, 4) == "SIZE" )
      {
        specified_channel_count = static_cast<int>(st.size() - 1);

        // Allocate enough memory to accommodate all fields
        if( specified_channel_count != static_cast<int> (cloud.fields.size ()) )
        {
          throw "The number of elements in <SIZE> differs than the number of "
                "elements in <FIELDS>!";
        }

        // Resize to accommodate the number of values
        field_sizes.resize(specified_channel_count);

        int offset = 0;
        for(int i = 0; i < specified_channel_count; ++i)
        {
          int col_type ;
          sstream >> col_type;
          cloud.fields[i].offset = offset;  // estimate and save the offsets
          offset += col_type;
          field_sizes[i] = col_type;        // save a temporary copy
        }
        cloud.point_step = offset;
        //if (cloud.width != 0)
          //cloud.row_step   = cloud.point_step * cloud.width;
      }

      // Get the field types
      else if( line_type.substr(0, 4) == "TYPE" )
      {
        if( field_sizes.empty() )
        {
          throw "TYPE of FIELDS specified before SIZE in header!";
        }

        specified_channel_count = static_cast<int>(st.size() - 1);

        // Allocate enough memory to accommodate all fields
        if( specified_channel_count != static_cast<int>(cloud.fields.size()) )
        {
          throw "The number of elements in <TYPE> differs than the number of "
                "elements in <FIELDS>!";
        }

        // Resize to accommodate the number of values
        field_types.resize(specified_channel_count);

        for(int i = 0; i < specified_channel_count; ++i)
        {
          field_types[i] = st.at (i + 1).c_str()[0];
          cloud.fields[i].datatype = 
            static_cast<uint8_t>(pcl::getFieldType(field_sizes[i],
                                                   field_types[i]));
        }
      }

      // Get the field counts
      else if( line_type.substr(0, 5) == "COUNT" )
      {
        if( field_sizes.empty () || field_types.empty () )
        {
          throw "COUNT of FIELDS specified before SIZE or TYPE in header!";
        }

        specified_channel_count = static_cast<int>(st.size() - 1);

        // Allocate enough memory to accommodate all fields
        if(specified_channel_count != static_cast<int>(cloud.fields.size()) )
        {
          throw "The number of elements in <COUNT> differs than the number of "
                "elements in <FIELDS>!";
        }

        field_counts.resize(specified_channel_count);

        int offset = 0;
        for(int i = 0; i < specified_channel_count; ++i)
        {
          cloud.fields[i].offset = offset;
          int col_count;
          sstream >> col_count;
          cloud.fields[i].count = col_count;
          offset += col_count * field_sizes[i];
        }

        // Adjust the offset for count (number of elements)
        cloud.point_step = offset;
      }

      // Get the width of the data (organized point cloud dataset)
      else if( line_type.substr(0, 5) == "WIDTH" )
      {
        sstream >> cloud.width;
        if( cloud.point_step != 0 )
        {
          // row_step only makes sense for organized datasets
          cloud.row_step = cloud.point_step * cloud.width;
        }
      }

      // Get the height of the data (organized point cloud dataset)
      else if( line_type.substr(0, 6) == "HEIGHT" )
      {
        sstream >> cloud.height;
      }

      // Check the format of the acquisition viewpoint
      else if( line_type.substr(0, 9) == "VIEWPOINT" )
      {
         if( st.size() < 8 )
         {
          throw "Not enough number of elements in <VIEWPOINT>! Need 7 values "
                "(tx ty tz qw qx qy qz).";
         }

        float x, y, z, w;
        sstream >> x >> y >> z ;
        origin      = Eigen::Vector4f(x, y, z, 0.0f);
        sstream >> w >> x >> y >> z;
        orientation = Eigen::Quaternionf(w, x, y, z);
      }

      // Get the number of points
      else if( line_type.substr(0, 6) == "POINTS" )
      {
        sstream >> nr_points;
        // Need to allocate: N * point_step
        cloud.data.resize(nr_points * cloud.point_step);
      }

      // Last field: read the header + comments line by line until we get to
      // <DATA>
      else if( line_type.substr(0, 4) == "DATA" )
      {
        if( st.at(1).substr(0, 17) == "binary_compressed" )
        {
          data_type = 2;
        }
        else if( st.at(1).substr(0, 6) == "binary" )
        {
          data_type = 1;
        }

        break;
      }
    }
  }

  catch( const char *exception)
  {
    LOGERROR("[pcl::PCDReader2::readHeader] %s", exception);
    return -1;
  }

  // Exit early: if no points have been given, there's no sense to read or
  // check anything anymore
  if( nr_points == 0 )
  {
    LOGERROR("[pcl::PCDReader2::readHeader] No points to read.");
    return -1;
  }
  
  // Compatibility with older PCD file versions
  if( cloud.width == 0 && cloud.height == 0 )
  {
    cloud.width  = nr_points;
    cloud.height = 1;
    // row_step only makes sense for organized datasets
    cloud.row_step = cloud.point_step * cloud.width;
  }

  //assert (cloud.row_step != 0);
  // If row_step = 0, either point_step was not set or width is 0

  // if both height/width are not given, assume an unorganized dataset
  if( cloud.height == 0 )
  {
    cloud.height = 1;
    LOGDIAG1("Warning: [pcl::PCDReader2::readHeader] no HEIGHT given, "
             "setting to 1 (unorganized).");
    if( cloud.width == 0 )
    {
      cloud.width  = nr_points;
    }
  }
  else
  {
    if( cloud.width == 0 && nr_points != 0 )
    {
      LOGERROR("[pcl::PCDReader2::readHeader] HEIGHT given (%d) but no WIDTH!",
          cloud.height);
      return -1;
    }
  }

  if( (int)(cloud.width * cloud.height) != nr_points )
  {
    LOGERROR("[pcl::PCDReader2::readHeader] HEIGHT (%d) x WIDTH (%d) "
              "!= number of points (%d)",
              cloud.height, cloud.width, nr_points);
    return -1;
  }

  return (ssize_t)sizeHdr;
}

ssize_t EuPcdIo::readbufDataAscii(byte_t data[],
                                  size_t        sizeData,
                                  //pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud)
                         pcl::PCLPointCloud2 &cloud)
{
  unsigned int  idx     = 0;
  size_t        offset  = 0;

  // get the number of points the cloud should have
  unsigned int nr_points = cloud.width * cloud.height;

  // setting the is_dense property to true by default
  cloud.is_dense = true;

  if( sizeData == 0 )
  {
    LOGERROR("[pcl::PCDReader2::readbuf] Data length is zero.");
    return -1;
  }

  std::string line;
  std::vector<std::string> st;

  // read the rest of the file
  try
  {
    while( (idx < nr_points) && (offset < sizeData) )
    {
      offset += EuPcdIo::getline(data+offset, sizeData-offset, line);

      // ignore empty lines
      if( line == "" )
      {
        continue;
      }

      // tokenize the line
      boost::trim(line);
      boost::split(st, line, boost::is_any_of("\t\r "),
                             boost::token_compress_on);
        
      if (idx >= nr_points)
      {
        LOGDIAG1("Warning: [pcl::PCDReader2::readbuf] input buffer has more "
                 "points (%d) than advertised (%d)!",
                    idx, nr_points);
        break;
      }

      size_t total = 0;

      // copy data
      for(unsigned int d = 0;
          d < static_cast<unsigned int>(cloud.fields.size());
          ++d)
      {
        // ignore invalid padded dimensions that are inherited from binary data
        if( cloud.fields[d].name == "_" )
        {
          // jump over this many elements in the string token
          total += cloud.fields[d].count;
          continue;
        }
        for(unsigned int c = 0; c < cloud.fields[d].count; ++c)
        {
          switch(cloud.fields[d].datatype)
          {
            case pcl::PCLPointField::INT8:
            {
              pcl::copyStringValue<pcl::traits::asType<pcl::PCLPointField::INT8>::type>(
                  st.at(total + c), cloud, idx, d, c);
              break;
            }
            case pcl::PCLPointField::UINT8:
            {
              pcl::copyStringValue<pcl::traits::asType<pcl::PCLPointField::UINT8>::type>(
                    st.at(total + c), cloud, idx, d, c);
              break;
            }
            case pcl::PCLPointField::INT16:
            {
              pcl::copyStringValue<pcl::traits::asType<pcl::PCLPointField::INT16>::type>(
                    st.at(total + c), cloud, idx, d, c);
              break;
            }
            case pcl::PCLPointField::UINT16:
            {
              pcl::copyStringValue<pcl::traits::asType<pcl::PCLPointField::UINT16>::type>(
                    st.at(total + c), cloud, idx, d, c);
              break;
            }
            case pcl::PCLPointField::INT32:
            {
              pcl::copyStringValue<pcl::traits::asType<pcl::PCLPointField::INT32>::type>(
                    st.at(total + c), cloud, idx, d, c);
              break;
            }
            case pcl::PCLPointField::UINT32:
            {
              pcl::copyStringValue<pcl::traits::asType<pcl::PCLPointField::UINT32>::type>(
                    st.at(total + c), cloud, idx, d, c);
              break;
            }
            case pcl::PCLPointField::FLOAT32:
            {
              pcl::copyStringValue<pcl::traits::asType<pcl::PCLPointField::FLOAT32>::type>(
                    st.at(total + c), cloud, idx, d, c);
              break;
            }
            case pcl::PCLPointField::FLOAT64:
            {
              pcl::copyStringValue<pcl::traits::asType<pcl::PCLPointField::FLOAT64>::type>(
                    st.at(total + c), cloud, idx, d, c);
              break;
            }
            default:
              LOGDIAG1("Warning: [pcl::PCDReader2::readbuf] incorrect field "
                       "data type specified (%d)!",
                          cloud.fields[d].datatype);
              break;
          }
        }
        // jump over this many elements in the string token
        total += cloud.fields[d].count;
      }
      idx++;
    }
  }

  catch(const char *exception)
  {
    LOGERROR("[pcl::PCDReader2::readbuf] %s", exception);
    return -1;
  }

  if( idx != nr_points )
  {
    LOGERROR("[pcl::PCDReader2::readbuf] Number of points read (%d) is "
              "different than expected (%d)", idx, nr_points);
    return -1;
  }

  return (ssize_t)offset;
}

ssize_t EuPcdIo::readbufDataBinary(byte_t data[],
                                   size_t        sizeData,
                                   //pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud)
                         pcl::PCLPointCloud2 &cloud)
{
  // get the number of points the cloud should have
  unsigned int nr_points = cloud.width * cloud.height;

  // setting the is_dense property to true by default
  cloud.is_dense = true;

  if( sizeData == 0 )
  {
    LOGERROR("[pcl::PCDReader2::readbuf] Data length is zero.");
    return -1;
  }

  // Copy the data
  memcpy(&cloud.data[0], data, sizeData); //cloud.data.size());

  int point_size = static_cast<int>(cloud.data.size () /
                              (cloud.height * cloud.width));


  //
  // Once copied, we need to go over each field and check if it has NaN/Inf
  // values and assign cloud.is_dense to true or false.
  //
  for(uint32_t i = 0; i < cloud.width * cloud.height; ++i)
  {
    for(unsigned int d = 0;
        d < static_cast<unsigned int>(cloud.fields.size());
        ++d)
    {
      for(uint32_t c = 0; c < cloud.fields[d].count; ++c)
      {
        switch( cloud.fields[d].datatype )
        {
          case pcl::PCLPointField::INT8:
          {
            if( !pcl::isValueFinite<pcl::traits::asType<pcl::PCLPointField::INT8>::type>(cloud, i, point_size, d, c) )
              cloud.is_dense = false;
            break;
          }
          case pcl::PCLPointField::UINT8:
          {
            if( !pcl::isValueFinite<pcl::traits::asType<pcl::PCLPointField::UINT8>::type>(cloud, i, point_size, d, c) )
              cloud.is_dense = false;
            break;
          }
          case pcl::PCLPointField::INT16:
          {
            if( !pcl::isValueFinite<pcl::traits::asType<pcl::PCLPointField::INT16>::type>(cloud, i, point_size, d, c) )
              cloud.is_dense = false;
            break;
          }
          case pcl::PCLPointField::UINT16:
          {
            if( !pcl::isValueFinite<pcl::traits::asType<pcl::PCLPointField::UINT16>::type>(cloud, i, point_size, d, c) )
              cloud.is_dense = false;
            break;
          }
          case pcl::PCLPointField::INT32:
          {
            if( !pcl::isValueFinite<pcl::traits::asType<pcl::PCLPointField::INT32>::type>(cloud, i, point_size, d, c) )
              cloud.is_dense = false;
            break;
          }
          case pcl::PCLPointField::UINT32:
          {
            if( !pcl::isValueFinite<pcl::traits::asType<pcl::PCLPointField::UINT32>::type>(cloud, i, point_size, d, c) )
              cloud.is_dense = false;
            break;
          }
          case pcl::PCLPointField::FLOAT32:
          {
            if( !pcl::isValueFinite<pcl::traits::asType<pcl::PCLPointField::FLOAT32>::type>(cloud, i, point_size, d, c) )
              cloud.is_dense = false;
            break;
          }
          case pcl::PCLPointField::FLOAT64:
          {
            if( !pcl::isValueFinite<pcl::traits::asType<pcl::PCLPointField::FLOAT64>::type>(cloud, i, point_size, d, c) )
              cloud.is_dense = false;
            break;
          }
        }
      }
    }
  }

  return (ssize_t)sizeData;
}

size_t EuPcdIo::getline(byte_t buf[], size_t sizeBuf, std::string &line)
{
  const size_t    MaxLine = 256;

  size_t    n;

  line.clear();

  for(n=0; n<sizeBuf; ++n)
  {
    if( buf[n] == '\n' )
    {
      ++n;
      break;
    }
  }

  if( n > 0 )
  {
    line.assign((char *)buf, n);
  }

  return n;
}

#endif // EU_HAS_PCL
#endif // EU_OLD_1_6
