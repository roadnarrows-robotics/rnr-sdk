#
# RoadNarrows Robotics Software Development Kit Dependencies
#   (development version)
#

# librnr package
find_library(LIBRNR
  NAMES rnr
  PATHS /prj/lib/rnr
)

# gpio package
find_library(LIBGPIO
  NAMES rnr_gpio
  PATHS /prj/lib/rnr
)

# i2c package
find_library(LIBI2C
  NAMES rnr_i2c
  PATHS /prj/lib/rnr
)

# libserial package
find_library(LIBSERIAL
  NAMES rnr_serial
  PATHS /prj/lib/rnr
)

# netmsgs package
find_library(LIBNETMSGS
  NAMES rnr_netmsgs
  PATHS /prj/lib/rnr
)

# botsense package
find_library(LIBBOTSENSE
  NAMES botsense
  PATHS /prj/lib
)

# appkit packages
find_library(LIBAPPKIT
  NAMES rnr_appkit
  PATHS /prj/lib/rnr
)

find_library(LIBTINYXML
  NAMES rnr_tinyxml
  PATHS /prj/lib/rnr
)

# peripherals packages
find_library(LIBHID
  NAMES rnr_hid
  PATHS /prj/lib/rnr
)

find_library(LIBIMU
  NAMES rnr_imu
  PATHS /prj/lib/rnr
)

# Dynamixel packages
find_library(LIBDXL
  NAMES rnr_dxl
  PATHS /prj/lib/rnr
)

find_library(LIBDYNAMIXEL
  NAMES rnr_dynamixel
  PATHS /prj/lib/rnr
)

set(rnr-sdk_LIBRARIES 
  ${LIBDYNAMIXEL}
  ${LIBDXL}
  ${LIBAPPKIT}
  ${LIBTINYXML}
  ${LIBIMU}
  ${LIBHID}
  ${LIBBOTSENSE}
  ${LIBNETMSGS}
  ${LIBSERIAL}
  ${LIBI2C}
  ${LIBGPIO}
  ${LIBRNR}
)

set(rnr-sdk_INCLUDE_DIRS /prj/include)
