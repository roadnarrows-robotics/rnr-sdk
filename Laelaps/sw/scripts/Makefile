################################################################################
#
# ./sw/scripts/Makefile

ifdef RNMAKE_DOXY
/*!
\file

$LastChangedDate: 2016-03-17 12:14:21 -0600 (Thu, 17 Mar 2016) $  
$Rev: 4350 $ 

\brief Make Laelaps executable scripts.

RN Make System Specific Makefile

\author: Robin Knight (robin.knight@roadnarrows.com)

##! \par Copyright
##!   \h_copy 2015-2017. RoadNarrows LLC.\n
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
# Executable Scripts

SCRIPTS = \
	laelaps_init.d.py \
	laelaps_service

#	laelaps_diag.py 
#	laelaps_tune_motors.py

#------------------------------------------------------------------------------
# Sources

#------------------------------------------------------------------------------
# Optional Variables 
#

# Subpackage C PreProcessor Flags
EXTRA_TGT_ALL = all-bin-scripts

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
# Extras

DISTDIR_BIN_SCRIPTS	:= $(addprefix $(DISTDIR_BIN)/,$(SCRIPTS))

.PHONY: all-bin-scripts
all-bin-scripts: $(DISTDIR_BIN_SCRIPTS)

$(DISTDIR_BIN)/% : %
	@printf "\n"
	@printf "$(color_tgt_file)    cp $(<) $(@)$(color_end)\n"
	@$(CP) $(<) $(@)


ifdef RNMAKE_DOXY
/*! \endcond RNMAKE_DOXY */
endif
