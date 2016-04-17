################################################################################
#
# ./dynashell/Makefile

ifdef RNMAKE_DOXY
/*!
\file

$LastChangedDate: 2015-01-12 10:56:06 -0700 (Mon, 12 Jan 2015) $
$Rev: 3845 $

\brief Make dynashell simple shell.

RN Make System Specific Makefile

\author: Robin Knight (robin@rnisys.com)

\par Copyright:
  (C) 2011-2015.  RoadNarrows Intelligent Systems.
  (http://www.roadnarrows.com)
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
pkgroot		= ..

#------------------------------------------------------------------------------
# Programs

DIST_PGMS         = dynashell 

READLINE_LIBS     = readline ncurses pthread
READLINE_CPPFLAGS = -DHAVE_READLINE

# botsense modules are located in a sub-directory of lib
LIB_SUBDIR = $(PKG_LIB_PLUGIN_SUBDIR) $(libdir)/botsense

# Libraries to Link With
dynashell.LIBS      = Dynamixel dxl bsclient_dyna netmsgs botsense rnr \
											stdc++ $(READLINE_LIBS) gsl gslcblas m
dynashell.LIBDEPS   = Dynamixel dxl bsclient_dyna

#------------------------------------------------------------------------------
# Sources

#
# C Source Files
#
dynashell.SRC.CXX = \
	dynashell.cxx \
	dynashell_cmd.cxx \
	dynashell_cmd_core.cxx \
	dynashell_cmd_if.cxx \
	dynashell_cmd_os.cxx \
	dynashell_cmd_servo.cxx \
	dynashell_main.cxx \
	dynashell_readline.cxx \
	dynashell_recording.cxx \
	dynashell_regex.cxx

#	dynashell_cmd_train.cxx


#------------------------------------------------------------------------------
# Optional Variables 
#

# Subpackage C PreProcessor Flags
EXTRA_CPPFLAGS = -DHAVE_READLINE -DLOG -DLOGMOD="\"dynashell\""

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
