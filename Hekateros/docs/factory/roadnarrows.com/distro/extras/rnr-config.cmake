#
# Hekateros and Pan-Tilt RNR System Dependencies
#

find_library(LIBI2C
  NAMES i2c
  PATHS /prj/lib/rnr /usr/local/lib/rnr
  NO_DEFAULT_PATH
)

find_library(LIBIMU
  NAMES imu
  PATHS /prj/lib/rnr /usr/local/lib/rnr
  NO_DEFAULT_PATH
)

find_library(LIBMOT
  NAMES mot
  PATHS /prj/lib/rnr /usr/local/lib/rnr
  NO_DEFAULT_PATH
)

find_library(LIBHID
  NAMES hid
  PATHS /prj/lib/rnr /usr/local/lib/rnr
  NO_DEFAULT_PATH
)

find_library(LIBRNR
  NAMES rnr
  PATHS /prj/lib/rnr /usr/local/lib/rnr
  NO_DEFAULT_PATH
)

find_library(LIBAPPKIT
  NAMES rnr_appkit
  PATHS /prj/lib/rnr /usr/local/lib/rnr
  NO_DEFAULT_PATH
)

find_library(LIBSERIAL
  NAMES serial
  PATHS /prj/lib/rnr /usr/local/lib/rnr
  NO_DEFAULT_PATH
)

find_library(LIBBOTSENSE
  NAMES botsense
  PATHS /prj/lib /usr/local/lib
  NO_DEFAULT_PATH
)

find_library(LIBDXL
  NAMES dxl
  PATHS /prj/lib /usr/local/lib
  NO_DEFAULT_PATH
)

find_library(LIBDYNAMIXEL
  NAMES Dynamixel
  PATHS /prj/lib /usr/local/lib
  NO_DEFAULT_PATH
)

find_library(LIBHEKATEROS
  NAMES hekateros
  PATHS /prj/lib /usr/local/lib
  NO_DEFAULT_PATH
)

find_library(LIBNETMSGS
  NAMES netmsgs
  PATHS /prj/lib /usr/local/lib
  NO_DEFAULT_PATH
)

set(rnr_LIBRARIES 
  ${LIBHEKATEROS}
  ${LIBDYNAMIXEL}
  ${LIBDXL}
  ${LIBBOTSENSE}
  ${LIBNETMSGS}
  ${LIBHID}
  ${LIBIMU}
  ${LIBMOT}
  ${LIBI2C}
  ${LIBSERIAL}
  ${LIBAPPKIT}
  ${LIBRNR}
)

set(rnr_INCLUDE_DIRS /prj/include /usr/local/include NO_DEFAULT_PATH)
