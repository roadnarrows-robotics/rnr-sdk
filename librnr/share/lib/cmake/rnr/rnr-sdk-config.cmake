#
# RoadNarrows Robotics Software Development Kit Dependencies
#   (apt install official version)
#

# librnr package
find_library(LIBRNR
  NAMES rnr
  PATHS /usr/local/lib/rnr
)

# gpio package
find_library(LIBGPIO
  NAMES rnr_gpio
  PATHS /usr/local/lib/rnr
)

# i2c package
find_library(LIBI2C
  NAMES rnr_i2c
  PATHS /usr/local/lib/rnr
)

# libserial package
find_library(LIBSERIAL
  NAMES rnr_serial
  PATHS /usr/local/lib/rnr
)

# netmsgs package
find_library(LIBNETMSGS
  NAMES rnr_netmsgs
  PATHS /usr/local/lib/rnr
)

# botsense package
find_library(LIBBOTSENSE
  NAMES botsense
  PATHS /usr/local/lib/botsense
)

# appkit packages
find_library(LIBAPPKIT
  NAMES rnr_appkit
  PATHS /usr/local/lib/rnr
)

find_library(LIBTINYXML
  NAMES rnr_tinyxml
  PATHS /usr/local/lib/rnr
)

# peripherals packages
find_library(LIBHID
  NAMES rnr_hid
  PATHS /usr/local/lib/rnr
)

find_library(LIBIMU
  NAMES rnr_imu
  PATHS /usr/local/lib/rnr
)

# Dynamixel packages
find_library(LIBDXL
  NAMES rnr_dxl
  PATHS /usr/local/lib/rnr
)

find_library(LIBDYNAMIXEL
  NAMES rnr_dynamixel
  PATHS /usr/local/lib/rnr
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

set(rnr-sdk_INCLUDE_DIRS /usr/local/include)
