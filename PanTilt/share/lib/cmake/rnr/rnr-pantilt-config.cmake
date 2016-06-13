#
# Pan-Tilt RNR System Dependencies
#

find_library(LIBHID
  NAMES rnr_hid
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

find_library(LIBNETMSGS
  NAMES rnr_netmsgs
  PATHS /prj/lib/rnr /usr/local/lib/rnr
)

set(rnr-pantilt_LIBRARIES 
  ${LIBDYNAMIXEL}
  ${LIBDXL}
  ${LIBAPPKIT}
  ${LIBHID}
  ${LIBBOTSENSE}
  ${LIBNETMSGS}
  ${LIBSERIAL}
  ${LIBGPIO}
  ${LIBRNR}
)

set(rnr-pantilt_INCLUDE_DIRS /prj/include /usr/local/include)
