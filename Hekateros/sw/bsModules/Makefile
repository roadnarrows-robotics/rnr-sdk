################################################################################
#
# ./bsModules/Makefile

ifdef RNMAKE_DOXY
/*!
\file

$LastChangedDate: 2016-03-18 12:24:39 -0600 (Fri, 18 Mar 2016) $
$Rev: 4360 $

\brief
Make all of the Hekateros server and client plug-in modules and BotSense
python modules.

RN Make System Specific Makefile

\par Usage:
make [RNMAKE_OPTS] [has_botsense={yes|no}] target

\author: Robin Knight (robin.knight@roadnarrows.com)

##! \copyright
##!   \h_copy 2012-2017. RoadNarrows LLC.\n
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

#------------------------------------------------------------------------------
# Optional variables

# do [not] include botsense subpackage in package
ifeq "$(findstring $(has_botsense),n no 0 false)" ""
	has_botsense := yes
else
	has_botsense := no
endif

#------------------------------------------------------------------------------
# Subdirectories

# Subdirectories to Build
ifeq "$(has_botsense)" "yes"
SUBDIRS = bsHek
endif

#------------------------------------------------------------------------------
# Make Includes

# Default RoadNarrows make system base directory
ifndef rnmake
rnmake := $(realpath $(pkgroot)/../rnmake)
endif

# Include Rules Makefile
include $(rnmake)/Rules.mk


ifdef RNMAKE_DOXY
/*! \endcond RNMAKE_DOXY */
endif
