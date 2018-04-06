################################################################################
#
# Rules.swig.mk
#
ifdef RNMAKE_DOXY
/*! 
\file 

\brief Special rules file for swigging C into python.

Include this file in each appropriate local make file (usually near the bottom).

\par Key RNMAKE Variables:
	\li SWIG_FILES 				- list of swig *.i interface files.
	\li SWIG_EXTMOD_DIR		- output *.py and shared libraries directory.
	\li SWIG_EXTMOD_LIBS	- list of -l<lib> libraries to link in.

\pkgsynopsis
RN Make System

\pkgfile{Rules.swig.mk}

\pkgauthor{Robin Knight,robin.knight@roadnarrows.com}

\pkgcopyright{2010-2018,RoadNarrows LLC,http://www.roadnarrows.com}

\license{MIT}

\EulaBegin
\EulaEnd

\cond RNMAKE_DOXY
 */
endif
#
################################################################################

_RULES_SWIG_MK = 1

ifeq ($(strip $(RNMAKE_SWIG_ENABLED)),y)

# Simplified Wrapper and Interface Generator command
SWIG = /usr/bin/swig

SWIG_BASES		= $(basename $(SWIG_FILES))
SWIG_FILES_C 	=	$(addsuffix _wrap.c,$(SWIG_BASES))
SWIG_FILES_O 	=	$(addprefix $(OBJDIR)/,$(subst .c,.o,$(SWIG_FILES_C)))
SWIG_FILES_PY	=	$(addprefix $(SWIG_EXTMOD_DIR)/,$(addsuffix .py,$(SWIG_BASES)))
SWIG_EXTMODS 	= $(addprefix $(SWIG_EXTMOD_DIR)/_,\
								$(addsuffix $(SHLIB_SUFFIX),$(SWIG_BASES)))

# Get python version stripped of revision number.
# Examples outputs from python --version:
# 	Python 2.7.6
# 	Python 2.7.11+
PYTHON_VER	= $(shell python --version 2>&1 | sed -e 's/Python //' -e's/\.[0-9]*[\+]*$$//'  )

ifeq "$(RNMAKE_ARCH)" "cygwin"
SWIG_PYLIB	= -lpython$(PYTHON_VER).dll
else
SWIG_PYLIB	= -lpython$(PYTHON_VER)
endif

ifndef "$(SWIG_INCLUDES)"
	SWIG_INCLUDES = -I/usr/include/python$(PYTHON_VER)
endif


SWIG_LIBS			= $(SWIG_EXTMOD_LIBS) $(SWIG_PYLIB)

SWIG_DONE			= $(RNMAKE_ARCH).done

# C and link loader flags
CFLAGS 	:= $(EXTRA_CFLAGS) $(RNMAKE_PKG_CFLAGS) $(SWIG_CFLAGS)
LDFLAGS	:= $(EXTRA_LDFLAGS) $(RNMAKE_PKG_LDFLAGS) $(SWIG_LDFLAGS)

define cond-clean
endef

# Target:	swig-all (default)
.PHONY: swig-all
swig-all: echo-swig-all swig-pre swig-mods
	@$(TOUCH) $(SWIG_DONE)

.PHONY: echo-swig-all
echo-swig-all:
	$(call printEchoTgtGoalDesc,Creating python wrappers from C/C++ source)

# Target: swig-pre
# If targets were made for another architecture, then clean up so that targets
# will be remade for the target architecture. The file done.<arch> determines
# the last made target architecture.
#
.PHONY: swig-pre
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
%.$(RNMAKE_ARCH).done: $(SWIG_EXTMOD_DIR)/_%$(SHLIB_SUFFIX)
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
