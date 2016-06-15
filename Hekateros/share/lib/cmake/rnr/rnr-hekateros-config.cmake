#
# Hekateros RNR System Dependencies
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
  PATHS /prj/lib /usr/local/lib
)

find_library(LIBDXL
  NAMES rnr_dxl
  PATHS /prj/lib/rnr /usr/local/lib/rnr
)

find_library(LIBDYNAMIXEL
  NAMES rnr_dynamixel
  PATHS /prj/lib/rnr /usr/local/lib/rnr
)

find_library(LIBHEKATEROS
  NAMES hekateros
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

find_library(LIBMOT
  NAMES rnr_mot
  PATHS /prj/lib/rnr /usr/local/lib/rnr
)

set(rnr-hekateros_LIBRARIES 
  ${LIBHEKATEROS}
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

set(rnr-hekateros_INCLUDE_DIRS /prj/include /usr/local/include)
