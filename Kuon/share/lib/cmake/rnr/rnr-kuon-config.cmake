#
# Kuon RNR System Dependencies
#

find_library(LIBHID
  NAMES rnr_hid
  PATHS /prj/lib/rnr /usr/local/lib/rnr
)

find_library(LIBI2C
  NAMES rnr_i2c
  PATHS /prj/lib/rnr /usr/local/lib/rnr
)

find_library(LIBRNR
  NAMES rnr
  PATHS /prj/lib/rnr /usr/local/lib/rnr
)

find_library(LIBAPPKIT
  NAMES rnr_appkit
  PATHS /prj/lib/rnr /usr/local/lib/rnr
)

find_library(LIBSERIAL
  NAMES rnr_serial
  PATHS /prj/lib/rnr /usr/local/lib/rnr
)

find_library(LIBGPIO
  NAMES rnr_gpio
  PATHS /prj/lib/rnr /usr/local/lib/rnr
)

find_library(LIBBOTSENSE
  NAMES botsense
  PATHS /prj/lib/botsense /usr/local/lib/botsense
)

find_library(LIBDXL
  NAMES rnr_dxl
  PATHS /prj/lib/rnr /usr/local/lib/rnr
)

find_library(LIBDYNAMIXEL
  NAMES rnr_dynamixel
  PATHS /prj/lib/rnr /usr/local/lib/rnr
)

find_library(KUON
  NAMES kuon
  PATHS /prj/lib /usr/local/lib
)

find_library(LIBNETMSGS
  NAMES rnr_netmsgs
  PATHS /prj/lib/rnr /usr/local/lib/rnr
)

find_library(LIBIMU
  NAMES rnr_imu
  PATHS /prj/lib/rnr /usr/local/lib/rnr
)

set(rnr-kuon_LIBRARIES 
  ${KUON}
  ${LIBDYNAMIXEL}
  ${LIBDXL}
  ${LIBAPPKIT}
  ${LIBHID}
  ${LIBIMU}
  ${LIBMOT}
  ${LIBBOTSENSE}
  ${LIBNETMSGS}
  ${LIBI2C}
  ${LIBSERIAL}
  ${LIBGPIO}
  ${LIBRNR}
)

set(rnr-kuon_INCLUDE_DIRS /prj/include /usr/local/include)
