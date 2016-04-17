################################################################################
#
# Package: 	RN Make System 
#
# File:			Cmds.mk
#
ifdef RNMAKE_DOXY
/*! 
\file 

\brief Host commands independent of target architecture.

$LastChangedDate: 2010-07-19 11:40:37 -0600 (Mon, 19 Jul 2010) $
$Rev: 502 $

\author Robin Knight (robin.knight@roadnarrows.com)

\par Copyright:
(C) 2005-2009.  RoadNarrows LLC.
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

# Force make to use /bin/sh
SHELL = /bin/sh

# Common file operations
MKDIR         = mkdir -p -m 775
CHMOD         = chmod
INSTALL_DIR   = install -d -p -m 775
INSTALL_EXE   = install -p -m 775
INSTALL       = install -p -m 664
CP						= cp -fp
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
