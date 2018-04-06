################################################################################
#
# ./sw/librnr_wingtk/Makefile
#
ifdef RNMAKE_DOXY
/*!
\file

\brief The librnr_wingtk library makefile.

RN Make System Specific Makefile

\pkgsynopsis
RoadNarrows Robotics Application Kit

\pkgfile{sw/librnr_wingtk/Makefile}

\pkgauthor{Robin Knight,robin.knight@roadnarrows.com} 
\pkgauthor{Daniel Packard,daniel@roadnarrows.com} 

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
RNMAKE_PKG_ROOT	= ../..

# Bootstrap package within RN Make System
include $(RNMAKE_PKG_ROOT)/make/Bootstrap.mk


#------------------------------------------------------------------------------
# Libraries

# Distribution Static Libraries
RNMAKE_DIST_STLIBS = rnr_wingtk

# Distribution Shared Libraries
RNMAKE_DIST_SHLIBS = rnr_wingtk

# Target library subdirectory
rnr_wingtk.SUBDIR = rnr

# Source Files
rnr_wingtk.SRC.CXX = \
	WinGtk.cxx \
	WinGtkMenu.cxx

#	WinGtkIoI.cxx \
#	WinGtkMenu.cxx \
#	WinGtkMouse.cxx \
#	WinGtkMsg.cxx \
#	WinGtkUtil.cxx 

# Library Package Dependencies
rnr_wingtk.LIBDEPS =	rnr_win

# Linked Libraries 
rnr_wingtk.LIBS =	\
	rnr_win rnr \
	$(OPENCV_COM_LIBS) \
  $(GTK_LIB) $(GDK_LIB) $(GDK_PIXBUF_LIB) $(ATK_LIB) $(PANGO_LIBS) \
  $(CAIRO_LIB) $(FONT_LIBS) $(GOBJ_LIBS) \
  $(GST_LIB_IF) $(GLIB_LIB) \
	stdc++

#opencv_core opencv_highgui opencv_imgproc \
#									jpeg \
#									rnr \
#									stdc++ \
#									pthread \
#									gtk-x11-2.0 gdk-x11-2.0 atk-1.0 gio-2.0 \
#									pangoft2-1.0 gdk_pixbuf-2.0 m pangocairo-1.0 \
#									cairo pango-1.0 \
#									freetype fontconfig \
#									gobject-2.0 gmodule-2.0 gthread-2.0 rt glib-2.0


#------------------------------------------------------------------------------
# Extras

# Subpackage C PreProcessor Flags
EXTRA_CPPFLAGS = -DLOG -DLOGMOD="\"librnr_wingtk\""

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
