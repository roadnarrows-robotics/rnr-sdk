////////////////////////////////////////////////////////////////////////////////
/*! \file
 *
 * \brief Package installation information wrapper.
 *
 * \note Add entries as new architectures are supported.
 *
 * \warning The install.h approach is obsolete. Need to replace with a better
 * system.
 *
 * \pkgsynopsis
 * RoadNarrows Robotics Common Library 1
 *
 * \pkgcomponent{Library}
 * librnr
 *
 * \pkgfile{rnr/install.h}
 *
 * \author Robin Knight (robin.knight@roadnarrows.com)
 *
 * \pkgcopyright{2010-2018,RoadNarrows LLC.,http://www.roadnarrows.com}
 *
 * \license{MIT}
 *
 * \EulaBegin
 * See the README and EULA files for any copyright and licensing information.
 * \EulaEnd
 */
////////////////////////////////////////////////////////////////////////////////

#ifndef _RNR_INSTALL_WRAPPER_H
#define _RNR_INSTALL_WRAPPER_H

#warning "The rnr/install.h file is deprecated."

#include "rnr/rnrconfig.h"

C_DECLS_BEGIN

#if 0 // deprecated

#if defined(ARCH_i386)

# include "arch/arch.i386/install.h"

#elif defined(ARCH_x86_64)

# include "arch/arch.x86_64/install.h"

#elif defined(ARCH_armang)

# include "arch/arch.armang/install.h"

#elif defined(ARCH_armpxa)

# include "arch/arch.armpxa/install.h"

#elif defined(ARCH_overo)

# include "arch/arch.overo/install.h"

#elif defined(ARCH_cygwin)

# include "arch/arch.cygwin/install.h"

#elif defined(ARCH_osx)

# include "arch/arch.osx/install.h"

#elif defined(ARCH_phony)

# include "arch/arch.phony/install.h"

#elif defined(ARCH_linaro)

# include "arch/arch.linaro/install.h"

#elif defined(ARCH_odroid)

# include "arch/arch.odroid/install.h"

#else

# warning "Unknown architecture"

#endif // deprecated

//
// Default installation
//

/*! \brief package install bin directory*/
#define PKG_INSTALL_BINDIR      "/usr/local/bin"

/*! \brief package install system bin directory */
#define PKG_INSTALL_SBINDIR     "/usr/local/sbin"

/*! \brief package install include directory */
#define PKG_INSTALL_INCDIR      "/usr/local/include"

/*! \brief package library directory */
#define PKG_INSTALL_LIBDIR      "/usr/local/lib64"

/*! \brief package install system configuration directory */
#define PKG_INSTALL_SYSCONFDIR  "/usr/local/etc"

/*! \brief package install documentation directory */
#define PKG_INSTALL_DOCDIR      "/usr/local/share/doc"

/*! \brief package install man pages directory */
#define PKG_INSTALL_MANDIR      "/usr/local/man"

/*! \brief package install information directory (s) */
#define PKG_INSTALL_INFODIR     "/usr/local/info"
#endif    // end supported architectures

C_DECLS_END


#endif // _RNR_INSTALL_WRAPPER_H
