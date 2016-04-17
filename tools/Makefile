################################################################################
#
# Makefile
#
# Version:
# 	$LastChangedDate: 2016-02-23 14:42:44 -0700 (Tue, 23 Feb 2016) $
# 	$Rev: 4330 $
#
# Description:
#   Make RoadNarrows Tools to Manage Projects, Packages, and the Business.
#
# Author: Robin Knight (robin.knight@roadnarrows.com)
#
# Copyright:
# (C) 2008-2016.  RoadNarrows LLC
# All Rights Reserved
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
# IN NO EVENT SHALL THE AUTHOR, ROADNARROWS LLC , OR ANY
# MEMBERS/EMPLOYEES OF ROADNARROW LLC OR DISTRIBUTORS OF THIS
# SOFTWARE BE LIABLE TO ANY PARTY FOR DIRECT, INDIRECT, SPECIAL, INCIDENTAL,
# OR CONSEQUENTIAL DAMAGES ARISING OUT OF THE USE OF THIS SOFTWARE AND ITS
# DOCUMENTATION, EVEN IF THE AUTHORS OR ANY OF THE ABOVE PARTIES HAVE BEEN
# ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
#
# THE AUTHOR AND ROADNARROWS LLC SPECIFICALLY DISCLAIM ANY
# WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
# MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE. THE SOFTWARE PROVIDED
# HEREUNDER IS ON AN "AS IS" BASIS, AND THE AUTHORS AND DISTRIBUTORS HAVE NO
# OBLIGATION TO PROVIDE MAINTENANCE, SUPPORT, UPDATES, ENHANCEMENTS, OR
# MODIFICATIONS.
#
################################################################################

scripts = \
	bfg \
	dpkg_repro_status \
	dpkg_repro_update \
	eula_update.py \
	fastinstall \
	git_pkg_status.sh \
	git_pull_all.sh \
	gitpurge \
	prod_configure \
	prod_unconfigure \
	rnmake_deb_all \
	rnmake_all_arch \
	rnmake_lsarch \
	rnmake_pkgnew \
	rnupload2spool \
	rnupload-dpkg \
	rpc_repro_status \
	sgrep \
	svn_co_sw \
	svn_pkg_status.sh \
	svn_update_all.sh
	unsvn.sh


bindir = /prj/bin

install:
	@cp $(scripts) $(bindir)/.
