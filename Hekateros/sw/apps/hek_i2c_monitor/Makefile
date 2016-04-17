################################################################################
#
# ./sw/apps/hek_i2c_monitor/Makefile

ifdef RNMAKE_DOXY
/*!
\file

$LastChangedDate: 2014-09-18 16:53:49 -0600 (Thu, 18 Sep 2014) $
$Rev: 3748 $

\brief
Make GFS-Monitor and hekateros limits applications.

RN Make System Specific Makefile

\author: Robin Knight (robin.knight@RoadNarrows.com)
\author: Brent Wilkins (Brent@RoadNarrows.com)

\par Copyright:
  (C) 2012-2013.  RoadNarrows Intelligent Systems.
  (http://www.RoadNarrows.com)
  All Rights Reserved

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
pkgroot		= ../../..


#------------------------------------------------------------------------------
# Programs

DIST_PGMS = i2c-monitor \
            hek_limits

# Libraries to Link With
i2c-monitor.LIBS = i2c rnr

hek_limits.LIBS = i2c rnr


#------------------------------------------------------------------------------
# Sources

#
# C Source Files
#
i2c-monitor.SRC.C = i2c-monitor.c
hek_limits.SRC.C = hek_limits.c

#------------------------------------------------------------------------------
# Optional Variables 
#

# Subpackage C PreProcessor Flags
i2c-monitor::EXTRA_CPPFLAGS = -DLOG -DLOGMOD="\"i2c-monitor\""
hek_limits::EXTRA_CPPFLAGS = -DLOG -DLOGMOD="\"hek_limits\""

#------------------------------------------------------------------------------
# Make Includes


#------------------------------------------------------------------------------
# Sources

#
# C Source Files
#
i2c-monitor.SRC.C = i2c-monitor.c
hek_limits.SRC.C = hek_limits.c

#------------------------------------------------------------------------------
# Optional Variables 
#

# Subpackage C PreProcessor Flags
i2c-monitor::EXTRA_CPPFLAGS = -DLOG -DLOGMOD="\"i2c-monitor\""
hek_limits::EXTRA_CPPFLAGS = -DLOG -DLOGMOD="\"hek_limits\""

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
