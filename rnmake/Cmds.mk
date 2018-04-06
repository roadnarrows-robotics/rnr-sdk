################################################################################
#
# Cmds.mk
#
ifdef RNMAKE_DOXY
/*! 
\file 

\brief Host commands independent of target architecture.

\pkgsynopsis
RN Make System

\pkgfile{Cmds.mk}

\pkgauthor{Robin Knight,robin.knight@roadnarrows.com}

\pkgcopyright{2005-2018,RoadNarrows LLC,http://www.roadnarrows.com}

\license{MIT}

\EulaBegin
\EulaEnd

\cond RNMAKE_DOXY
 */
endif
#
################################################################################

_CMDS_MK = 1

# Force make to use /bin/sh
SHELL = /bin/sh

# Common file operations
MKDIR         = mkdir -p -m 775
CHMOD         = chmod
INSTALL_DIR   = install -d -p -m 775
INSTALL_EXE   = install -p -m 775
INSTALL       = install -p -m 664
CP						= cp -fp
CP_R					= $(CP) -r
RM						= rm -fr
RMFILE				= rm -f
MV						= mv
ifeq "$(ARCH)" "osx"
TAR						= tar -c -z -v -f
else
TAR						= tar --create --gzip --atime-preserve --verbose --file
endif
SED						= sed -r
FIND					= find
BASENAME			= basename
SYMLINK				= ln -s
UNLINK				= unlink
GREP					= grep
FILE					= file
XARGS					= xargs
TOUCH					= touch

# Interpreters
PERL          = /usr/bin/perl
PYTHON        = /usr/bin/python

# LEX and YACC
LEX           = flex
YACC          = bison -y


ifdef RNMAKE_DOXY
/*! \endcond RNMAKE_DOXY */
endif
