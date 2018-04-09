################################################################################
#
# Prod.mk
#
# Description:
#  	RN Make System Package Makefile
#
#  	@prod_fqname@ Product Makefile.
#
# Author(s): @rndiv_fqname@ (@rndiv_email@)
#
# Copyright (C) @THIS_YEAR@.  @rndiv_fqname@
# All Rights Reserved
#
# @THIS_DATE@ @THIS_TIME@ 
#
# @rndiv_eula@
#
################################################################################

# Prevent mutliple inclusion
RROD_MK						= 1

ifndef topdir
$(error Error: topdir not defined in including makefile)
endif

# The Product Definition
PROD                 	= @prod_name@
PROD_VERSION_MAJOR   	= @prod_ver_major@
PROD_VERSION_MINOR   	= @prod_ver_minor@
PROD_VERSION_RELEASE 	= @prod_ver_rev@
PROD_VERSION_SPEC 		= @prod_ver_spec@
PROD_VERSION_DATE    	= @THIS_YEAR@
PROD_OWNERS          	= "@rndiv_fqname@"
PROD_VERSION_DOTTED  	= $(PROD_VERSION_MAJOR).$(PROD_VERSION_MINOR).$(PROD_VERSION_RELEASE)$(PROD_VERSION_SPEC)
PROD_VERSION_CAT    	= $(PROD_VERSION_MAJOR)$(PROD_VERSION_MINOR)$(PROD_VERSION_RELEASE)
PROD_FULL_NAME       	= $(PROD)-$(PROD_VERSION_DOTTED)

#------------------------------------------------------------------------------
# Optional Variables and Tweaks

# install prefix root
prefix_root						= $(topdir)/xinstall

#
# Packages to Build
#
PKGS = @pkg_list@

# top include directories to resolve package inter-dependencies
PROD_INCDIRS = $(addprefix $(topdir)/,$(addsuffix /include,$(PKGS)))
