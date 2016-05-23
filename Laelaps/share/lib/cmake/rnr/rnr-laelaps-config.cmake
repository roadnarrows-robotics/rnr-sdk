#
# Laelaps RNR System Dependencies
#

find_library(LIBHID
  NAMES hid
  PATHS /prj/lib/rnr /usr/local/lib/rnr
)

find_library(LIBI2C
  NAMES i2c
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
  NAMES serial
  PATHS /prj/lib/rnr /usr/local/lib/rnr
)

find_library(LIBGPIO
  NAMES gpio
  PATHS /prj/lib/rnr /usr/local/lib/rnr
)

find_library(LIBBOTSENSE
  NAMES botsense
  PATHS /prj/lib /usr/local/lib
)

find_library(LIBDXL
  NAMES dxl
  PATHS /prj/lib /usr/local/lib
)

find_library(LIBDYNAMIXEL
  NAMES Dynamixel
  PATHS /prj/lib /usr/local/lib
)

find_library(LIBLAELAPS
  NAMES laelaps
  PATHS /prj/lib /usr/local/lib
)

find_library(LIBNETMSGS
  NAMES netmsgs
  PATHS /prj/lib /usr/local/lib
)

find_library(LIBIMU
  NAMES imu
  PATHS /prj/lib/rnr /usr/local/lib/rnr
)


set(rnr-laelaps_LIBRARIES 
  ${LIBLAELAPS}
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

set(rnr-laelaps_INCLUDE_DIRS /prj/include /usr/local/include)
