################################################################################
#
# ./sw/apps/hek_looking_at_you/Makefile

ifdef RNMAKE_DOXY
/*!
\file

$LastChangedDate: 2014-09-18 16:53:49 -0600 (Thu, 18 Sep 2014) $  
$Rev: 3748 $ 

\brief
Make hek_looking_at_you application.

RN Make System Specific Makefile

\author: Robin Knight (robin@rnisys.com)

##! \copyright
##!   \h_copy 2011-2017. RoadNarrows LLC.\n
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
pkgroot		= ../../..


#------------------------------------------------------------------------------
# Programs

#DIST_PGMS         = hek_looking_at_you 

# Libraries to Link With
hek_looking_at_you.LIBS  = \
	opencv_core \
	opencv_highgui \
	opencv_imgproc \
	jpeg \
	rnrwin-gtk \
	rnr \
	stdc++

hek_looking_at_you.LIBDEPS   = Dynamixel


#------------------------------------------------------------------------------
# Sources

#
# C Source Files
#
hek_looking_at_you.SRC.CXX = ilay.cxx

#------------------------------------------------------------------------------
# Optional Variables 
#

# Subpackage C PreProcessor Flags
EXTRA_CPPFLAGS = -DLOG -DLOGMOD="\"hek_looking_at_you\""

ifeq "$(arch)" "overo"
EXTRA_INCDIRS  = /opt/xc/overo/include/gtk-2.0 \
								 /opt/xc/overo/lib/gtk-2.0/include \
								 /opt/xc/overo/include/pango-1.0 \
								 /opt/xc/overo/include/cairo \
								 /opt/xc/overo/include/glib-2.0 \
								 /opt/xc/overo/lib/glib-2.0/include \
								 /opt/xc/overo/include/atk-1.0
else
EXTRA_INCDIRS  = /usr/include/gtk-2.0 \
								 /usr/lib/gtk-2.0/include \
								 /usr/include/pango-1.0 \
								 /usr/include/cairo \
								 /usr/include/glib-2.0 \
								 /usr/lib/glib-2.0/include \
								 /usr/include/atk-1.0
endif

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
