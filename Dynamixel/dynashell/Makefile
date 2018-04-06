################################################################################
#
# ./dynashell/Makefile
#
ifdef RNMAKE_DOXY
/*!
\file

\brief The dynashell application makefile.

RN Make System Specific Makefile

\pkgsynopsis
RoadNarrows Robotics \h_dynamixel Package

\pkgfile{dynashell/Makefile}

\pkgauthor{Robin Knight,robin.knight@roadnarrows.com} 

\pkgcopyright{2011-2018,RoadNarrows LLC,http://www.roadnarrows.com}

\license{MIT}

\EulaBegin
\EulaEnd

 * \cond RNMAKE_DOXY
 */
endif
# 
################################################################################

#------------------------------------------------------------------------------
# Required

# Package Root Directory
RNMAKE_PKG_ROOT	= ..

# Bootstrap package within RN Make System
include $(RNMAKE_PKG_ROOT)/make/Bootstrap.mk


#------------------------------------------------------------------------------
# Programs

RNMAKE_DIST_PGMS = dynashell 

READLINE_LIBS     = readline ncurses pthread
READLINE_CPPFLAGS = -DHAVE_READLINE

# Libraries to Link With
dynashell.LIBS = \
	rnr_dynamixel \
	rnr_dxl \
	bsclient_dyna \
	rnr_netmsgs \
	botsense \
	rnr \
	stdc++ $(READLINE_LIBS) gsl gslcblas m

dynashell.LIBDEPS  = rnr_dynamixel rnr_dxl bsclient_dyna

# C++ Source Files
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
# Extras

# install botsense library directory
EXTRA_LD_LIBDIRS = $(libdir)/botsense

# Subpackage C PreProcessor Flags
EXTRA_CPPFLAGS = -DHAVE_READLINE -DLOG -DLOGMOD="\"dynashell\""


#------------------------------------------------------------------------------
# Include RNMAKE rules makefile(s)

# include top-level rules
include $(RNMAKE_ROOT)/Rules.mk


ifdef RNMAKE_DOXY
/*! \endcond RNMAKE_DOXY */
endif
