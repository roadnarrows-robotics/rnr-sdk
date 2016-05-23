#
# Pan-Tilt RNR System Dependencies
#

find_library(LIBHID
  NAMES hid
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

find_library(LIBNETMSGS
  NAMES netmsgs
  PATHS /prj/lib /usr/local/lib
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
