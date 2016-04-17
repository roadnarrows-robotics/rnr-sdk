#
# Hekateros/Pan-Tilt RNR System Dependencies
#

find_library(LIBHID
  NAMES hid
  PATHS /usr/local/lib/rnr /prj/lib/rnr
)

find_library(LIBI2C
  NAMES i2c
  PATHS /usr/local/lib/rnr /prj/lib/rnr
)

find_library(LIBRNR
  NAMES rnr
  PATHS /usr/local/lib/rnr /prj/lib/rnr
)

find_library(LIBAPPKIT
  NAMES rnr_appkit
  PATHS /usr/local/lib/rnr /prj/lib/rnr
)

find_library(LIBSERIAL
  NAMES serial
  PATHS /usr/local/lib/rnr /prj/lib/rnr
)

find_library(LIBBOTSENSE
  NAMES botsense
  PATHS /usr/local/lib /prj/lib
)

find_library(LIBDXL
  NAMES dxl
  PATHS /usr/local/lib /prj/lib
)

find_library(LIBDYNAMIXEL
  NAMES Dynamixel
  PATHS /usr/local/lib /prj/lib
)

find_library(LIBHEKATEROS
  NAMES hekateros
  PATHS /usr/local/lib /prj/lib
)

find_library(LIBNETMSGS
  NAMES netmsgs
  PATHS /usr/local/lib /prj/lib
)

find_library(LIBRS160D
  NAMES RS160D
  PATHS /usr/local/lib /prj/lib
)

find_library(LIBIMU
  NAMES imu
  PATHS /usr/local/lib/rnr /prj/lib/rnr
)

find_library(LIBMOT
  NAMES mot
  PATHS /usr/local/lib/rnr /prj/lib/rnr
)


set(rnr_LIBRARIES 
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
  ${LIBRNR}
)

set(rnr_INCLUDE_DIRS /usr/local/include /prj/include)
