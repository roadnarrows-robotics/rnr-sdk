################################################################################
#
# Colors.mk
#
ifdef RNMAKE_DOXY
/*! 
\file 

\brief Color schemes.

\pkgsynopsis
RN Make System

\pkgfile{Colors.mk}

\pkgauthor{Robin Knight,robin.knight@roadnarrows.com}

\pkgcopyright{2012-2018,RoadNarrows LLC,http://www.roadnarrows.com}

\license{MIT}

\EulaBegin
\EulaEnd

\cond RNMAKE_DOXY
 */
endif
#
################################################################################

_COLORS_MK = 1

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
