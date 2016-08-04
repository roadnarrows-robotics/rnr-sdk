#
# Eudoxus RNR System Dependencies
#

find_library(LIBRNR
  NAMES rnr
  PATHS /prj/lib/rnr /usr/local/lib/rnr
)

find_library(LIBI2C
  NAMES rnr_i2c
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

find_library(LIBAPPKIT
  NAMES rnr_appkit
  PATHS /prj/lib/rnr /usr/local/lib/rnr
)

set(rnr-eudoxus_LIBRARIES 
  ${LIBRNR}
  ${LIBI2C}
  ${LIBSERIAL}
  ${LIBGPIO}
  ${LIBAPPKIT}
)

set(rnr-eudoxus_INCLUDE_DIRS /prj/include /usr/local/include)
