################################################################################
#
# Package: 	RN Make System 
#
# File:			Rules.swig.mk
#
ifdef RNMAKE_DOXY
/*! 
\file 

\brief Special rules file for swigging C into python.

Include this file into each local make file (usually at the bottom).\n

\par Key Variables:
	\li SWIG_FILES 				- list of swig *.i interface files.
	\li SWIG_EXTMOD_DIR		- output *.py and shared libraries directory.
	\li SWIG_EXTMOD_LIBS	- list of -l<lib> libraries to link in.

$LastChangedDate: 2012-11-05 11:05:42 -0700 (Mon, 05 Nov 2012) $
$Rev: 2507 $

\author Robin Knight (robin.knight@roadnarrows.com)

\par Copyright:
(C) 2010.  RoadNarrows LLC.
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

ifeq "$(SWIG_ENABLED)" "y"

# Simplified Wrapper and Interface Generator command
SWIG = /usr/bin/swig

SWIG_BASES		= $(basename $(SWIG_FILES))
SWIG_FILES_C 	=	$(addsuffix _wrap.c,$(SWIG_BASES))
SWIG_FILES_O 	=	$(addprefix $(OBJDIR)/,$(subst .c,.o,$(SWIG_FILES_C)))
SWIG_FILES_PY	=	$(addprefix $(SWIG_EXTMOD_DIR)/,$(addsuffix .py,$(SWIG_BASES)))
SWIG_EXTMODS 	= $(addprefix $(SWIG_EXTMOD_DIR)/_,\
								$(addsuffix $(SHLIB_SUFFIX),$(SWIG_BASES)))

PYTHON_VER	= $(shell python --version 2>&1 | sed -e 's/Python //' -e's/\.[0-9]*$$//'  )

ifeq "$(ARCH)" "cygwin"
SWIG_PYLIB	= -lpython$(PYTHON_VER).dll
else
SWIG_PYLIB	= -lpython$(PYTHON_VER)
endif

ifndef "$(SWIG_INCLUDES)"
	SWIG_INCLUDES = -I/usr/include/python$(PYTHON_VER)
endif


SWIG_LIBS			= $(SWIG_EXTMOD_LIBS) $(SWIG_PYLIB)

SWIG_DONE			= $(ARCH).done

# C and link loader flags
CFLAGS 	:= $(EXTRA_CFLAGS) $(PKG_CFLAGS) $(SWIG_CFLAGS)
LDFLAGS	:= $(EXTRA_LDFLAGS) $(PKG_LDFLAGS) $(SWIG_LDFLAGS)

define cond-clean
endef

# Target:	swig-all (default)
swig-all: swig-pre swig-mods
	@$(TOUCH) $(SWIG_DONE)

# Target: swig-pre
# If targets were made for another architecture, then clean up so that targets
# will be remade for the target architecture. The file done.<arch> determines
# the last made target architecture.
#
swig-pre:
	@if [ ! -f $(SWIG_DONE) ]; \
	then \
		if [ -f *.done ]; \
		then \
			printf "Cleaning previous target: $(basename $(wildcard *.done)).\n"; \
			$(RM) *.done; \
		fi; \
		$(RM) $(OBJDIR); \
		$(RM) $(SWIG_FILES_C); \
		$(RM) $(SWIG_FILES_PY); \
		$(RM) $(SWIG_EXTMODS); \
	fi

# Target: swig-mods
# Make all swig modules
swig-mods: $(SWIG_EXTMODS)

# Target:	swig-clean
swig-clean:
	$(RM) $(SWIG_FILES_C)
	$(RM) $(SWIG_FILES_PY)
	$(RM) $(SWIG_EXTMODS)
	$(RM) *.done

# Done
# Make determines dependency sequence only for the first file (*.i) and the end
# file. Itermediates are not factored in. But rnmake supports cross-compiling,
# so the <name>.<arch>.done file is used as an artificial end file to force
# compliling.
#
%.$(ARCH).done: $(SWIG_EXTMOD_DIR)/_%$(SHLIB_SUFFIX)
	$(RM) *.done
	$(TOUCH) $(@)

# Swig Architecture-Specific Shared Library Rule:
$(SWIG_EXTMOD_DIR)/_%$(SHLIB_SUFFIX) : $(OBJDIR)/%_wrap.o
	@printf "\n"
	@printf "$(color_tgt_file)     $(<)$(color_end)\n"
	$(SHLIB_LD) $(LDFLAGS) $(LD_LIBPATHS) $(<) $(SWIG_LIBS) -o $(@)

# Swig C Rule: <name>.c -> $(OBJDIR)/<name>.o
$(OBJDIR)/%.o : %.c
	$(mkobjdir)
	@printf "\n"
	@printf "$(color_tgt_file)     $(<)$(color_end)\n"
	$(CC) $(CFLAGS) $(CPPFLAGS) $(INCLUDES) $(SWIG_INCLUDES) -o $(@) -c $(<)

# Swig I Rule: <name>.i -> <name>_wrap.c, <outdir>/<name>.py
%_wrap.c : %.i
	$(mkobjdir)
	@printf "\n"
	@printf "$(color_tgt_file)     $(<)$(color_end)\n"
	$(SWIG) -python $(INCLUDES) $(SWIG_INCLUDES) -v -outdir $(SWIG_EXTMOD_DIR) \
		-o $(@) $(<)

# don't autodelete intermediate files
.SECONDARY: $(SWIG_FILES_C) $(SWIG_FILES_O) $(SWIG_EXTMODS)

else

swig-all swig-clean:
	@printf "swig not enabled - ignoring\n"

endif

ifdef RNMAKE_DOXY
/*! \endcond RNMAKE_DOXY */
endif
