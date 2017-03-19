################################################################################
#
# ./libDynamixel/Makefile

ifdef RNMAKE_DOXY
/*!
\file

$LastChangedDate: 2015-03-13 13:28:02 -0600 (Fri, 13 Mar 2015) $
$Rev: 3890 $

\brief Make libDynamixel library.

RN Make System Specific Makefile

\author: Robin Knight (robin.knight@roadnarrows.com)

##! \copyright
##!   \h_copy 2011-2017. RoadNarrows LLC.\n
##!   http://www.roadnarrows.com\n
##!   All Rights Reserved

\cond RNMAKE_DOXY
 */
endif

# @EulaBegin@
# 
# Unless otherwise stated explicitly, all materials contained are copyrighted
# and may not be used without RoadNarrows LLC's written consent,
# except as provided in these terms and conditions or in the copyright
# notice (documents and software) or other proprietary notice provided with
# the relevant materials.
# 
# IN NO EVENT SHALL THE AUTHOR, ROADNARROWS LLC, OR ANY 
# MEMBERS/EMPLOYEES/CONTRACTORS OF ROADNARROWS OR DISTRIBUTORS OF THIS SOFTWARE
# BE LIABLE TO ANY PARTY FOR DIRECT, INDIRECT, SPECIAL, INCIDENTAL, OR
# CONSEQUENTIAL DAMAGES ARISING OUT OF THE USE OF THIS SOFTWARE AND ITS
# DOCUMENTATION, EVEN IF THE AUTHORS OR ANY OF THE ABOVE PARTIES HAVE BEEN
# ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
# 
# THE AUTHORS AND  ROADNARROWS LLC SPECIFICALLY DISCLAIM ANY WARRANTIES,
# INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND
# FITNESS FOR A PARTICULAR PURPOSE. THE SOFTWARE PROVIDED HEREUNDER IS ON AN
# "AS IS" BASIS, AND THE AUTHORS AND DISTRIBUTORS HAVE NO OBLIGATION TO
# PROVIDE MAINTENANCE, SUPPORT, UPDATES, ENHANCEMENTS, OR MODIFICATIONS.
# 
# @EulaEnd@
#
################################################################################

#------------------------------------------------------------------------------
# Required

# Package Root Directory
pkgroot		= ..

#------------------------------------------------------------------------------
# Libraries

# place libraries in subdirectory
LIB_SUBDIR = $(PKG_LIB_SUBDIR)

# Distribution Static Libraries
DIST_STLIBS 			= rnr_dynamixel

# Distribution Shared Libraries
DIST_SHLIBS 			= rnr_dynamixel

# Source Files
rnr_dynamixel.SRC.CXX	=	\
	DynaBgThread.cxx \
	DynaChain.cxx \
	DynaComm.cxx \
	DynaCommBotSense.cxx \
	DynaCommSerial.cxx \
	DynaError.cxx \
	DynaPid.cxx \
	DynaPidPos.cxx \
	DynaPidSpeed.cxx \
	DynaServo.cxx \
	DynaServoGeneric.cxx \
	DynaServoAX12.cxx \
	DynaServoEX106P.cxx \
	DynaServoMX12W.cxx \
	DynaServoMX28.cxx \
	DynaServoMX64.cxx \
	DynaServoMX106.cxx \
	DynaServoRX10.cxx \
	DynaServoRX24F.cxx \
	DynaServoRX28.cxx \
	DynaServoRX64.cxx

# Libraries to link with
rnr_dynamixel.LIBS = \
	rnr_dxl \
	bsclient_dyna \
	botsense \
	rnr_netmsgs \
	rnr_serial \
	rnr_gpio \
	rnr \
	pthread rt

rnr_dynamixel.LIBDEPS = rnr_dxl


#------------------------------------------------------------------------------
# Interface Headers


#------------------------------------------------------------------------------
# Optional Variables 
#

# Link Library Extra Library Directories (exluding local library)
EXTRA_LD_LIBDIRS = $(DISTROOT_LIB)/botsense

# Subpackage C PreProcessor Flags
EXTRA_CPPFLAGS = -DLOG -DLOGMOD="\"librnr_dynamixel\""

#------------------------------------------------------------------------------
# Make Includes
#

# Default RoadNarrows make system base directory
ifndef rnmake
rnmake := $(realpath $(pkgroot)/../rnmake)
endif

# Include Rules Makefile
include $(rnmake)/Rules.mk

#------------------------------------------------------------------------------
# Extra make targets (place after Rules.mk)


ifdef RNMAKE_DOXY
/*! \endcond RNMAKE_DOXY */
endif
