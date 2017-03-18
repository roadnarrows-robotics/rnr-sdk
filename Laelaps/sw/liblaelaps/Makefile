################################################################################
#
# ./sw/libraton/Makefile

ifdef RNMAKE_DOXY
/*!
\file

$LastChangedDate: 2016-01-15 12:44:03 -0700 (Fri, 15 Jan 2016) $
$Rev: 4260 $

\brief Make the liblaelaps library.

RN Make System Specific Makefile

\author: Robin Knight <robin.knight@roadnarrows.com>
\author: Daniel Packard <daniel@roadnarrows.com>

##! \par Copyright
##!   \h_copy 2013-2017. RoadNarrows LLC.\n
##!   http://www.roadnarrows.com\n
##!   All Rights Reserved

\cond RNMAKE_DOXY
 */
endif

#
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
pkgroot		= ../..

SUBDIRS = ut

#------------------------------------------------------------------------------
# Libraries

# Distribution Static Libraries
DIST_STLIBS    = laelaps

# Distribution Shared Libraries
DIST_SHLIBS    = laelaps

# Source Files
laelaps.SRC.CXX  = \
	laeAlarms.cxx \
	laeBatt.cxx \
	laeDb.cxx \
	laeDesc.cxx \
	laeGpio.cxx \
	laeI2C.cxx \
	laeI2CMux.cxx \
	laeImu.cxx \
	laeKin.cxx \
	laeMotor.cxx \
	laePlatform.cxx \
	laePowertrain.cxx \
	laeReports.cxx \
	laeRobot.cxx \
	laeThread.cxx \
	laeThreadAsync.cxx \
	laeThreadImu.cxx \
	laeThreadKin.cxx \
	laeThreadRange.cxx \
	laeThreadWd.cxx \
	laeTraj.cxx \
	laeTune.cxx \
	laeUtils.cxx \
	laeVL6180.cxx \
	laeWd.cxx \
	laeXmlCfg.cxx \
	laeXmlTune.cxx \
	RoboClaw.cxx

# Linked Libraries 
laelaps.LIBS	= \
	botsense \
	rnr_dynamixel \
	rnr_netmsgs \
	rnr_gpio \
	rnr_i2c \
	rnr_serial \
	rnr_appkit \
	rnr_tinyxml \
	rnr \
	pthread \
	stdc++

#------------------------------------------------------------------------------
# Optional Variables 
#

# Subpackage C PreProcessor Flags
EXTRA_CPPFLAGS = -DLOG -DLOGMOD="\"liblaelaps\"" -DLOGMOD_COLOR=LOG_COLOR_BLUE

#------------------------------------------------------------------------------
# Make Includes
#

# Default RoadNarrows make system base directory
ifndef rnmake
rnmake := $(realpath $(pkgroot)/../rnmake)
endif

# Include Rules Makefile
include $(rnmake)/Rules.mk

ifdef RNMAKE_DOXY
/*! \endcond RNMAKE_DOXY */
endif

