################################################################################
#
# Arch/Arch.atmega32uc3.mk
#
ifdef RNMAKE_DOXY
/*! 
\file 

\brief RoadNarrows Make System architecture makefile.

Make file for the Atmel 32-bit AVR UC3 MCU devices.

\par Architecture:
Atmel ATmega32uc3 Microcontroller

\par Build Host:
Cross-Compiler

\par Tool-Chain:
avr32-*

\par Usage:
make arch=atmega32uc3 [mcu=device] \<rnmake-target\>\n
	micro-contoller unit default (part number): uc3a1512

\pkgsynopsis
RN Make System

\pkgfile{Arch/Arch.atmega32uc3.mk}

\pkgauthor{Robin Knight,robin.knight@roadnarrows.com}

\pkgcopyright{2011-2018,RoadNarrows LLC,http://www.roadnarrows.com}

\license{MIT}

\EulaBegin
\EulaEnd

\cond RNMAKE_DOXY
 */
endif
#
################################################################################

# Prevent mutliple inclusion
_ARCH_ATMEGA32UC3_MK = 1

# This architecture (required)
RNMAKE_ARCH         = atmega32uc3
RNMAKE_ARCH_FQNAME	= atmega32uc3-mcu

# Micro-Controller Unit
mcu ?= uc3a1512
MCU = $(mcu)

# Output format. One of: srec, ihex, binary
# Default is ihex. Override in including make file if necessary.
FORMAT ?= ihex

# RDK TODO  map fuses
# Device High Fuse Byte
# Override in including make file if necessary.
#
# Field   Bit  Description                Default
# -----   ---  -----------                -------
# OCDEN    7   Enable OCD                 1 (unprogrammed, OCD disabled)
# JTAGEN   6   Enable JTAG                0 (programmed, JTAG enabled)
# SPIEN    5   Enable SPI Serial Program  0 (programmed, SPI prog. enabled)
#              and data Downloading
# CKOPT    4   Oscillator options         1 (unprogrammed)
# EESAVE   3   EEPROM memory is preserved 1 (unprogrammed, EEPROM not preserved)
#              through the Chip Erase
# BOOTSZ1  2   Select Boot Size           0 (programmed)
# BOOTSZ0  1   Select Boot Size           0 (programmed)
# BOOTRST  0   Select reset vector        1 (unprogrammed)
FUSE_HIGH ?= 0x99

# RDK TODO  map fuses
# Device Low Fuse Byte
# Override in including make file if necessary.
#
# Field   Bit  Description                Default
# -----   ---  -----------                -------
# BODLEVEL 7   Brown-out Detector trigger 1 (unprogrammed)
#              level
# BODEN    6   Brown-out Detector enable  1 (unprogrammed, BOD disabled)
# SUT1     5   Select start-up time       1 (unprogrammed)
# SUT0     4   Select start-up time       0 (programmed)
# CKSEL3   3 	 Select Clock source        0 (programmed)
# CKSEL2   2 	 Select Clock source        0 (programmed)
# CKSEL1   1 	 Select Clock source        0 (programmed)
# CKSEL0   0 	 Select Clock source        0 (unprogrammed)
FUSE_LOW ?= 0xE1


#------------------------------------------------------------------------------
# Tool Chain
#------------------------------------------------------------------------------

# Cross compiler tool chain prefix
CROSS_COMPILE       = avr32-

# Build Support Commands
AR                  = $(CROSS_COMPILE)ar
RANLIB              = $(CROSS_COMPILE)ranlib
STRIP_LIB						= $(CROSS_COMPILE)strip --strip-debug
STRIP_EXE						= $(CROSS_COMPILE)strip --strip-all
OBJCOPY 						= $(CROSS_COMPILE)objcopy
OBJDUMP 						= $(CROSS_COMPILE)objdump
SIZE 								= $(CROSS_COMPILE)size
NM 									= $(CROSS_COMPILE)nm


#------------------------------------------------------------------------------
# C Preprocessor Options
#------------------------------------------------------------------------------
RNMAKE_ARCH_INCDIRS  = /opt/avr32/include
RNMAKE_ARCH_CPPFLAGS =

#------------------------------------------------------------------------------
# Assembler and Options
#------------------------------------------------------------------------------
#AS                  = $(CROSS_COMPILE)as
AS                  = $(CC)

# Atmel device
ASFLAGS_MCU		= -mpart=$(MCU)

# Debugging format.
#  -gstabs:   Have the assembler create line number information; note that
#             for use in COFF files, additional information about filenames
#             and function names needs to be present in the assembler source
#             files -- see avr-libc docs [FIXME: not yet described there]
ASFLAGS_DEBUG 	= $(CFLAGS_DEBUG)

# Pass to other subprocesses
# -x Use CPP
ASFLAGS_PASS		= -x assembler-with-cpp

# Listing
#  -Wa,...:   tell GCC to pass this to the assembler.
#  -adlms:    create listing
ASFLAGS_LISTING 	= -Wa,-adhlns=$(<:.S=.lst)

# Assembler flags.
#  -Wa,...:   tell GCC to pass this to the assembler.
#  -adlms:    create listing
ARCH_ASFLAGS = 	$(ASFLAGS_MCU) \
								$(ASFLAGS_DEBUG) \
								$(ASFLAGS_PASS) \
								$(ASFLAGS_LISTING)


#------------------------------------------------------------------------------
# C Compiler and Options
#------------------------------------------------------------------------------
CC                  = $(CROSS_COMPILE)gcc

# Atmel device
CFLAGS_MCU						= -mpart=$(MCU)

# Compiler flags to generate dependency files.
# Not needed in rnmake system.
# -Wp		pass comma-separated options to cpp
#  -M		build dependency rule for object file
#  -MP	add .PHONY to each rule for object file
#  -MT	set output object to next argument
#  -MF	depencency rule output in file in next argument
#CFLAGS_GENDEPFLAGS = -Wp,-M,-MP,-MT,$(*F).o,-MF,.dep/$(@F).d

# Optimization level, can be [0, 1, 2, 3, s]. 
# 0 = turn off optimization. s = optimize for size.
# (Note: 3 is not always the best optimization level. See avr-libc FAQ.)
CFLAGS_OPTIMIZE_LEVEL = -O1

# Debugging format.
# Native formats for AVR-GCC's -g are stabs [default], or dwarf-2.
# AVR (extended) COFF requires stabs, plus an avr-objcopy run.
#CFLAGS_DEBUG = -gstabs
CFLAGS_DEBUG = -gdwarf-2

# Compiler flag to set the C Standard level.
# c89   - "ANSI" C
# gnu89 - c89 plus GCC extensions
# c99   - ISO C99 standard (not yet fully implemented)
# gnu99 - c99 plus GCC extensions
CFLAGS_CSTANDARD = -std=gnu99

# Optimizations
# characters are unsigned
# bitfields are unsigned
# pack structure members with no holes
# smallest enum posible
CFLAGS_OPTIMIZES =  -funsigned-char \
										-funsigned-bitfields \
										-fpack-struct \
										-fshort-enums

# Warnings
# All normal warnings
# Warn on strict prototypes
CFLAGS_WARNING  = -Wall -Wstrict-prototypes

# Listing
#  -Wa,...:   tell GCC to pass this to the assembler.
#  -adlms:    create listing
CFLAGS_LISTING 		= -Wa,-adhlns=$(<:.c=.lst)

CFLAGS_CPP_ONLY     = -E
CFLAGS_DEPS_ONLY    = -M

RNMAKE_ARCH_CFLAGS = 	$(CFLAGS_MCU) \
											$(CFLAGS_DEBUG) \
											$(CFLAGS_OPTIMIZE_LEVEL) \
                      $(CFLAGS_OPTIMIZES) \
                      $(CFLAGS_WARNING) \
                      $(CFLAGS_LISTING) \
											$(CFLAGS_CSTANDARD) \
											$(CFLAGS_GENDEPFLAGS)

# Make AS/C/CXX Dependencies Command
RNMAKE_MAKEDEPS	= $(CC) $(CFLAGS_DEPS_ONLY)


#------------------------------------------------------------------------------
# C++ Compiler and Options
#------------------------------------------------------------------------------
CXX                 = $(CROSS_COMPILE)g++
CXXFLAGS_CPP_ONLY   = -E
CXXFLAGS_DEPS_ONLY  = -M

RNMAKE_ARCH_CXXFLAGS =	$(CFLAGS_DEBUG) \
												$(CFLAGS_OPTIMIZE_LEVEL) \
                      	$(CFLAGS_OPTIMIZES) \
                      	$(CFLAGS_WARNING) \
                      	$(CFLAGS_OTHER) \
												$(CFLAGS_CSTANDARD)


#------------------------------------------------------------------------------
# Linker and Options
#------------------------------------------------------------------------------
LD									= $(CROSS_COMPILE)ld
LD_CC             	= $(LD)
LD_CXX              = $(LD)

# External memory options
#
# 64 KB of external RAM, starting after internal RAM (ATmega128!),
# used for variables (.data/.bss) and heap (malloc()).
#ARCH_LDFLAGS_EXTMEMOPTS = -Wl,-Tdata=0x801100,--defsym=__heap_end=0x80ffff
#
# 64 KB of external RAM, starting after internal RAM (ATmega128!),
# only used for heap (malloc()).
#ARCH_LDFLAGS_EXTMEMOPTS = -Wl,--defsym=__heap_start=0x801100,--defsym=__heap_end=0x80ffff

ARCH_LDFLAGS_EXTMEMOPTS =

# Linker flags.
#  -Wl,...:     tell GCC to pass this to linker.
#  -Map:      	create map file
#   --cref:    	add cross reference to  map file
#ARCH_LDFLAGS_LD		= -Wl,-Map=$(TARGET).map,--cref

ARCH_LDFLAGS			= $(ARCH_LDFLAGS_LD) \
										$(ARCH_LDFLAGS_EXTMEMOPTS)
ARCH_LD_LIBPATHS  = -L/opt/avr32/lib
ARCH_LD_LIBS      = 


#------------------------------------------------------------------------------
# Static Library Archiver and Options
#------------------------------------------------------------------------------
STLIB_LD            = ${AR} cr
STLIB_PREFIX        = lib
STLIB_SUFFIX        = .a


#------------------------------------------------------------------------------
# Other Useful Flags, Etc.
#------------------------------------------------------------------------------
# Minimalistic printf version
PRINTF_LIB_MIN = -Wl,-u,vfprintf -lprintf_min

# Floating point printf version (requires MATH_LIB = -lm below)
PRINTF_LIB_FLOAT = -Wl,-u,vfprintf -lprintf_flt

PRINTF_LIB = 

# Minimalistic scanf version
SCANF_LIB_MIN = -Wl,-u,vfscanf -lscanf_min

# Floating point + %[ scanf version (requires MATH_LIB = -lm below)
SCANF_LIB_FLOAT = -Wl,-u,vfscanf -lscanf_flt

SCANF_LIB = 

MATH_LIB = -lm


ifdef RNMAKE_DOXY
/*! \endcond RNMAKE_DOXY */
endif
