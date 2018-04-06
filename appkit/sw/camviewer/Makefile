################################################################################
#
# ./sw/camviewer/Makefile
#
ifdef RNMAKE_DOXY
/*!
\file

\brief The camviewer application makefile.

RN Make System Specific Makefile

\pkgsynopsis
RoadNarrows Robotics Application Kit

\pkgfile{sw/camviewer/Makefile}

\pkgauthor{Robin Knight,robin.knight@roadnarrows.com} 

\pkgcopyright{2013-2018,RoadNarrows LLC,http://www.roadnarrows.com}

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
RNMAKE_PKG_ROOT	= ../..

# Bootstrap package within RN Make System
include $(RNMAKE_PKG_ROOT)/make/Bootstrap.mk


#------------------------------------------------------------------------------
# Programs
RNMAKE_DIST_PGMS = camviewer

camviewer.SRC.CXX = camviewer.cxx

camviewer.LIBDEPS = rnr_wingtk rnr_winqt rnr_win rnr_cam rnr_appkit tinyxml
camviewer.LIBS 		= rnr_wingtk rnr_winqt rnr_win rnr_cam rnr_appkit rnr \
										rnr_tinyxml stdc++

#------------------------------------------------------------------------------
# Extras

# Subpackage C PreProcessor Flags
EXTRA_CPPFLAGS = -DLOG -DLOGMOD="\"camviewer\""

# Extra includes
EXTRA_INCDIRS = \
	$(GTK_INCDIR) \
	$(PANGO_INCDIR) \
	$(CAIRO_INCDIR) \
	$(ATK_INCDIR) \
	$(GDK_PIXBUF_INCDIR) \
	$(GTK_LIB_INCDIR) \
	$(GLIB_INCDIR) \
	$(GLIB_LIB_INCDIR)


#------------------------------------------------------------------------------
# Include RNMAKE rules makefile(s)

# include top-level rules
include $(RNMAKE_ROOT)/Rules.mk


ifdef RNMAKE_DOXY
/*! \endcond RNMAKE_DOXY */
endif
