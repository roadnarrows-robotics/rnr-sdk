################################################################################
#
# ./sw/librnr_appkit/ut/Makefile
#
ifdef RNMAKE_DOXY
/*!
\file

\brief The librnr_appkit unit tests makefile.

RN Make System Specific Makefile

\pkgsynopsis
RoadNarrows Robotics Application Kit

\pkgfile{sw/librnr_appkit/ut/Makefile}

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
RNMAKE_PKG_ROOT	= ../../..

# Bootstrap package within RN Make System
include $(RNMAKE_PKG_ROOT)/make/Bootstrap.mk

#------------------------------------------------------------------------------
# Programs

RNMAKE_TEST_PGMS = ut-librnr_appkit utThread utRegEx 

# Linked Libraries 
ut-librnr_appkit.LIBS = gtest rnr_appkit rnr pthread stdc++
utThread.LIBS					= rnr_appkit rnr pthread stdc++
utRegEx.LIBS  				= rnr_appkit rnr stdc++

ut-librnr_appkit.LIBDEPS 	= gtest rnr_appkit
utThread.LIBDEPS         	= rnr_appkit
utRegEx.LIBDEPS          	= rnr_appkit rnr stdc++

# C++ Source Files
ut-librnr_appkit.SRC.CXX 	= ut-librnr_appkit-main.cxx ut-sm.cxx
utThread.SRC.CXX 					= utThread.cxx
utRegEx.SRC.CXX 					= utRegEx.cxx


#------------------------------------------------------------------------------
# Extras

# Includes
EXTRA_SYS_INCDIRS	+= \
	$(GTK_INCDIR) \
	$(GTK_LIB_INCDIR) \
	$(GDK_PIXBUF_INCDIR) \
	$(PANGO_INCDIR) \
	$(CAIRO_INCDIR) \
	$(ATK_INCDIR) \
	$(GST_INCDIR) \
	$(LIBXML2_INCDIR) \
	$(GLIB_INCDIR) \
	$(GLIB_LIB_INCDIR)

# Subpackage C PreProcessor Flags
EXTRA_CPPFLAGS = -DLOG -DLOGMOD="\"UTlibrnr_appkit\""

EXTRA_INCDIRS += /opt/include


#------------------------------------------------------------------------------
# Include RNMAKE rules makefile(s)

# include top-level rules
include $(RNMAKE_ROOT)/Rules.mk


ifdef RNMAKE_DOXY
/*! \endcond RNMAKE_DOXY */
endif
