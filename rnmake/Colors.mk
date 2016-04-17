################################################################################
#
# Package: 	RN Make System 
#
# File:			Colors.mk
#
ifdef RNMAKE_DOXY
/*! 
\file 

\brief RoadNarrows Make Color Schemes.

$LastChangedDate: 2013-01-28 09:38:13 -0700 (Mon, 28 Jan 2013) $
$Rev: 2632 $

\author Robin Knight (robin.knight@roadnarrows.com)

\par Copyright:
(C) 2012.  RoadNarrows LLC.
(http://www.roadnarrows.com)
\n All Rights Reserved

\cond RNMAKE_DOXY
 */
endif
#
# Permission is hereby granted, without written agreement and without
# license or royalty fees, to use, copy, modify, and distribute this
# software and its documentation for any purpose, provided that
# (1) The above copyright notice and the following two paragraphs
# appear in all copies of the source code and (2) redistributions
# including binaries reproduces these notices in the supporting
# documentation.   Substantial modifications to this software may be
# copyrighted by their authors and need not follow the licensing terms
# described here, provided that the new terms are clearly indicated in
# all files where they apply.
#
# IN NO EVENT SHALL THE AUTHOR, ROADNARROWS LLC, OR ANY MEMBERS/EMPLOYEES
# OF ROADNARROW LLC OR DISTRIBUTORS OF THIS SOFTWARE BE LIABLE TO ANY
# PARTY FOR DIRECT, INDIRECT, SPECIAL, INCIDENTAL, OR CONSEQUENTIAL
# DAMAGES ARISING OUT OF THE USE OF THIS SOFTWARE AND ITS DOCUMENTATION,
# EVEN IF THE AUTHORS OR ANY OF THE ABOVE PARTIES HAVE BEEN ADVISED OF
# THE POSSIBILITY OF SUCH DAMAGE.
#
# THE AUTHOR AND ROADNARROWS LLC SPECIFICALLY DISCLAIM ANY WARRANTIES,
# INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND
# FITNESS FOR A PARTICULAR PURPOSE. THE SOFTWARE PROVIDED HEREUNDER IS ON AN
# "AS IS" BASIS, AND THE AUTHORS AND DISTRIBUTORS HAVE NO OBLIGATION TO
# PROVIDE MAINTENANCE, SUPPORT, UPDATES, ENHANCEMENTS, OR MODIFICATIONS.
#
################################################################################

ifeq "$(color)" ""
	color = default
endif

ifneq "$(color)" "off"

	color_pre   				= \033[
	color_post  				= \033[0m
	color_black   			= 0;30m
	color_red   				= 0;31m
	color_green   			= 0;32m
	color_yellow				= 0;33m
	color_blue   				= 0;34m
	color_magenta   		= 0;35m
	color_cyan   				= 0;36m
	color_white   			= 0;37m
	color_gray   			  = 1;30m
	color_light_red   	= 1;31m
	color_light_green   = 1;32m
	color_light_yellow	= 1;33m
	color_light_blue   	= 1;34m
	color_light_magenta	= 1;35m
	color_light_cyan   	= 1;36m
	color_bright_white  = 1;37m

	# fixed colors
	color_error					= $(color_pre)$(color_red)
	color_warn					= $(color_pre)$(color_yellow)

	# default color scheme
	ifeq "$(color)" "default"
		color_end 				= $(color_post)
		color_pkg_banner 	= $(color_pre)$(color_light_blue)
		color_dir_banner 	= $(color_pre)$(color_yellow)
		color_tgt_file 		= $(color_pre)$(color_green)
		color_tgt_lib 		= $(color_pre)$(color_cyan)
		color_tgt_pgm 		= $(color_pre)$(color_light_magenta)

	# neon color scheme
	else ifeq "$(color)" "neon"
		color_end 				= $(color_post)
		color_pkg_banner 	= $(color_pre)$(color_light_red)
		color_dir_banner 	= $(color_pre)$(color_light_magenta)
		color_tgt_file 		= $(color_pre)$(color_light_cyan)
		color_tgt_lib 		= $(color_pre)$(color_light_yellow)
		color_tgt_pgm 		= $(color_pre)$(color_light_yellow)

	# brazil color scheme
	else ifeq "$(color)" "brazil"
		color_end 				= $(color_post)
		color_pkg_banner 	= $(color_pre)$(color_green)
		color_dir_banner 	= $(color_pre)$(color_green)
		color_tgt_file 		= $(color_pre)$(color_light_yellow)
		color_tgt_lib 		= $(color_pre)$(color_blue)
		color_tgt_pgm 		= $(color_pre)$(color_blue)

	# whites color scheme
	else ifeq "$(color)" "whites"
		color_end 				= $(color_post)
		color_pkg_banner 	= $(color_pre)$(color_bright_white)
		color_dir_banner 	= $(color_pre)$(color_white)
		color_tgt_file 		= $(color_pre)$(color_gray)
		color_tgt_lib 		= $(color_pre)$(color_bright_white)
		color_tgt_pgm 		= $(color_pre)$(color_bright_white)

	else
$(warning Warning: $(color) scheme is unsupported.)

	endif
endif


ifdef RNMAKE_DOXY
/*! \endcond RNMAKE_DOXY */
endif
