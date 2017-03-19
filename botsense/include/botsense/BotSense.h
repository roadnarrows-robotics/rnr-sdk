////////////////////////////////////////////////////////////////////////////////
//
// Package:   BotSense
//
// File:      BotSense.h
//
/*! \file
 *
 * $LastChangedDate: 2011-11-09 08:46:08 -0700 (Wed, 09 Nov 2011) $
 * $Rev: 1506 $
 *
 * \brief \h_botsense package top-level, unifying header declarations.
 *
 * \note This file must be swig-able to generate a python extension module.
 *
 * \author Robin Knight (robin.knight@roadnarrows.com)
 *
 * \copyright
 *   \h_copy 2008-2017. RoadNarrows LLC.\n
 *   http://www.roadnarrows.com\n
 *   All Rights Reserved
 */
// Permission is hereby granted, without written agreement and without
// license or royalty fees, to use, copy, modify, and distribute this
// software and its documentation for any purpose, provided that
// (1) The above copyright notice and the following two paragraphs
// appear in all copies of the source code and (2) redistributions
// including binaries reproduces these notices in the supporting
// documentation.   Substantial modifications to this software may be
// copyrighted by their authors and need not follow the licensing terms
// described here, provided that the new terms are clearly indicated in
// all files where they apply.
//
// IN NO EVENT SHALL THE AUTHOR, ROADNARROWS LLC, OR ANY MEMBERS/EMPLOYEES
// OF ROADNARROW LLC OR DISTRIBUTORS OF THIS SOFTWARE BE LIABLE TO ANY
// PARTY FOR DIRECT, INDIRECT, SPECIAL, INCIDENTAL, OR CONSEQUENTIAL
// DAMAGES ARISING OUT OF THE USE OF THIS SOFTWARE AND ITS DOCUMENTATION,
// EVEN IF THE AUTHORS OR ANY OF THE ABOVE PARTIES HAVE BEEN ADVISED OF
// THE POSSIBILITY OF SUCH DAMAGE.
//
// THE AUTHOR AND ROADNARROWS LLC SPECIFICALLY DISCLAIM ANY WARRANTIES,
// INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND
// FITNESS FOR A PARTICULAR PURPOSE. THE SOFTWARE PROVIDED HEREUNDER IS ON AN
// "AS IS" BASIS, AND THE AUTHORS AND DISTRIBUTORS HAVE NO OBLIGATION TO
// PROVIDE MAINTENANCE, SUPPORT, UPDATES, ENHANCEMENTS, OR MODIFICATIONS.
//
////////////////////////////////////////////////////////////////////////////////

#ifndef _BOTSENSE_H
#define _BOTSENSE_H

#include "rnr/rnrconfig.h"

#ifndef SWIG
C_DECLS_BEGIN
#endif

/*!
 * \ingroup botsense_h
 * \defgroup bs_ecodes  BotSense Error Codes
 *
 * \h_botsense package-wide error codes.
 *
 * \{
 */
#define BS_OK                       0 ///< not an error, success

#define BS_ECODE_GEN                1 ///< general, unspecified error
#define BS_ECODE_BAD_RECV           2 ///< bad receive
#define BS_ECODE_BAD_SEND           3 ///< bad send
#define BS_ECODE_BAD_RESYNC         4 ///< bad resync with server/client
#define BS_ECODE_MSG_BAD_HDR        5 ///< bad message header
#define BS_ECODE_MSG_FRAG           6 ///< message fragment
#define BS_ECODE_MSG_TOO_BIG        7 ///< message too big
#define BS_ECODE_BUF_TOO_SMALL      8 ///< buffer too small
#define BS_ECODE_BAD_MSG            9 ///< bad message
#define BS_ECODE_BAD_VAL           10 ///< bad value
#define BS_ECODE_MSG_BAD_TID       11 ///< cannot match transaction id
#define BS_ECODE_BAD_VCONN_HND     12 ///< bad virtual connection handle
#define BS_ECODE_NO_VCONN          13 ///< virtual connection not found
#define BS_ECODE_UNKNOWN_REQ       14 ///< unknown request
#define BS_ECODE_NO_DEV            15 ///< no proxied device
#define BS_ECODE_NO_MOD            16 ///< no interface module
#define BS_ECODE_BAD_MOD           17 ///< bad interface module
#define BS_ECODE_NO_RSRC           18 ///< no resource available
#define BS_ECODE_BUSY              19 ///< resource busy
#define BS_ECODE_TIMEDOUT          20 ///< operation timed out
#define BS_ECODE_NO_EXEC           21 ///< cannot execute
#define BS_ECODE_SERVER_CONN_FAIL  22 ///< cannot connect to server
#define BS_ECODE_SERVER_CONN_DENY  23 ///< server denied connection
#define BS_ECODE_SERVER_BAD_CLIENT 24 ///< server detected bad client
#define BS_ECODE_SYS               25 ///< system (errno) error
#define BS_ECODE_INTERNAL          26 ///< internal error (bug)

#define BS_ECODE_BADEC             27 ///< bad error code

#define BS_ECODE_NUMOF             28 ///< number of error codes
/*! \} */

/*!
 * \ingroup botsense_h
 * \defgroup bs_uri  BotSense Uniform Resource Indentifier Attributes
 *
 * Applications may used a URI to specify local or BotSense proxied services.
 * See the RoadNarrows Dynamixel package for an example.
 * \{
 */
#define BSPROXY_URI_SCHEME        "botsense"      ///< URI scheme
#define BSPROXY_URI_HOSTNAME_DFT  "localhost"     ///< URI hostname default
/*! \} */

/*!
 * \ingroup botsense_h
 * \defgroup bs_tunes  BotSense bsProxy Server Compiled Default Tunables
 *
 * These defaults can be overridden in the botsense XML configuration file(s).
 *
 * \{
 */
#define BSPROXY_LISTEN_PORT_DFT  9195 ///< default bsProxy passive socket
#define BSPROXY_REG_CLIENT_MAX     16 ///< max number of simultaneous clients
#define BSPROXY_VCONN_CLIENT_MAX   16 ///< max number of virtual conn/client
#define BSPROXY_MSG_BODY_MAX     2048 ///< maximum msg body length (sans header)
/*! \} */


/*!
 * \ingroup botsense_h
 * \defgroup bs_vconn_handle  BotSense Handle Attributes
 *
 * Virtual connection handles provide a server unique identifier for virtual
 * connections between \h_botsense clients and opened interface module/proxied
 * device pairs. One special handle number is reserved for bsProxy server
 * terminated requests.
 *
 * \{
 */
#define BSPROXY_VCONN_MASK       0xff ///< virtual connection handle mask
#define BSPROXY_VCONN_UNDEF       255 ///< undefined virtual connection handle
#define BSPROXY_VCONN_SERVER      254 ///< handle for server-terminated msgs
#define BSPROXY_VCONN_MOD_MIN       0 ///< minimum module-specific handle value
#define BSPROXY_VCONN_MOD_MAX     253 ///< maximum module-specific handle value
#define BSPROXY_VCONN_MOD_NUMOF   254 ///< number of module-specific handles
#define BSPROXY_VCONN_NUMOF       (BSPROXY_VCONN_MOD_NUMOF+1)
                                      ///< total number of module handles
#define BSPROXY_VCONN_MIN         (BSPROXY_VCONN_MOD_MIN)
                                      ///< minimum handle value
#define BSPROXY_VCONN_MAX         (BSPROXY_VCONN_MOD_NUMOF)
                                      ///< maximum handle value

typedef int BsVConnHnd_T;             ///< virtual connection handle type
/*! \} */


/*!
 * \ingroup botsense_h
 * \defgroup bs_tid  BotSense Transaction Id Attributes
 *
 * A transaction id provides an identifier to map responses to requests. Since
 * a \h_botsense client can be mulit-threaded and the bsProxy server is multi-
 * threaded by proxied device, responses can be received by the client out of
 * order. The \h_botsense client library automatically reorders the responses
 * and delivers the correct response to the given request.
 *
 * \{
 */
#define BSPROXY_TID_MIN             0   ///< minimum transaction id
#define BSPROXY_TID_MAX           255   ///< maximum transaction id
#define BSPROXY_TID_NUMOF         256   ///< number of unique tid's
#define BSPROXY_TID_MASK         0xff   ///< transaction id mask

typedef uint_t BsTid_T;                 ///< client transaction id type [0-255].
/*! \} */


/*!
 * \ingroup botsense_h
 * \defgroup bs_msgid  BotSense Application-Specific Message Id Attributes
 *
 * A message id is a client virtual connection unique message identifier.
 * 16-bits are reserved in the \h_botsense header for upto 64k unique messages.
 *
 * \{
 */
#define BSPROXY_MSGID_NUMOF    0x10000  ///< number of vconn unique message ids
#define BSPROXY_MSGID_MASK      0xffff  ///< message id mask

typedef uint_t BsMsgId_T;               ///< client message id type [0-64k].
/*! \} */


/*!
 * \ingroup botsense_h
 * \defgroup bs_msguid  BotSense Server-Wide Message Unique Id Attributes
 *
 * A MSGUID is a server instance unique message id. It is the combination of the
 * server assigned virtual connection handle (or the reserved server handle)
 * and the application-specific message id.
 *
 * Although the MSGUID is not used by the \h_botsense client library nor by the
 * bsProxy server directly, it can be a useful construct for client applications
 * that need this level of uniqueness. The MSGUID is composed if two values:
 * \li Virtual connection handle.
 * \li Application-specific message id.
 *
 * \{
 */
#define BSPROXY_MSGUID_VCONN_SHIFT                16    ///< v.conn bit shift 
#define BSPROXY_MSGUID_VCONN_MASK  \
  (BSPROXY_VCONN_MASK << BSPROXY_MSGUID_VCONN_SHIFT)   ///< v.connection mask
#define BSPROXY_MSGUID_MSGID_MASK (BSPROXY_MSGID_MASK)  ///< app message id mask

/*!
 * \brief Make a MSGUID.
 * \param hndVConn  Virtual connection handle.
 * \param msgid     Application specific message id.
 */
#define BSPROXY_MAKE_MSGUID(hndVConn, msgid)  \
  (BsMsgUid_T)( \
      (((hndVConn) << BSPROXY_MSGUID_VCONN_SHIFT) & BSPROXY_MSGUID_VCONN_MASK) \
    | ((msgid) & BSPROXY_MSGUID_MSGID_MASK) )

/*!
 * \brief Make a server-specific MSGUID.
 * \param msgid   Application specific message id.
 */
#define BSPROXY_MAKE_SERVER_MSGUID(msgid) \
  BSPROXY_MAKE_MSGUID(BSPROXY_VCONN_SERVER, msgid)

/*!
 * \brief Get the virtual connection handle from the MSGUID.
 * \param msguid  MSGUID.
 */
#define BSPROXY_GET_MSGUID_HND(msguid) \
  (BsMsgUid_T)( \
      (((msguid) >> BSPROXY_MSGUID_VCONN_SHFIT) & BSPROXY_MSGUID_VCONN_MASK) )

/*!
 * \brief Get the message id from the MSGUID.
 * \param msguid  MSGUID.
 */
#define BSPROXY_GET_MSGUID_MSGID(msguid) \
  (BsMsgUid_T)( ((msguid) & BSPROXY_MSGUID_MSGID_MASK) )

typedef uint_t BsMsgUid_T;      ///< client message unique id
/*! \} */


/*!
 * \ingroup botsense_h
 * \defgroup bs_hdr  BotSense Message Header Attributes
 *
 * All \h_botsense server-directed message requests and client-directed
 * responses contain a fixed-format message header.
 *
 * \{
 */
#define BSPROXY_MSG_HDR_LEN   8           ///< message header length (bytes)
#define BSPROXY_MSG_MAX_LEN   (BSPROXY_MSG_HDR_LEN+BSPROXY_MSG_BODY_MAX)
                                          ///< total message maximum length
#define BSPROXY_MSG_MAGIC     0xaaaa      ///< message magic pattern

/*!
 * \brief Convenience macro to produce a buffer (offset, size) 2-tuple.
 *
 * The offset allows libbsclient to pack the header in front of the buffer
 * without doing any unnecessary copies, while the buffer size is decremented 
 * to account for the header bytes to be packed.
 *
 * \param buf   Buffer.
 */ 
#define BSPROXY_BUF_BODY(buf) \
   (buf)+BSPROXY_MSG_HDR_LEN, sizeof(buf)-(size_t)BSPROXY_MSG_HDR_LEN

/*!
 * \brief BotSense Proxy Message Header Structure
 */
typedef struct
{
  ushort_t        m_hdrMagic;   ///< "unique" magic pattern
  byte_t          m_hdrTid;     ///< transaction id
  byte_t          m_hdrVConn;   ///< virtual connection handle (server unique)
  ushort_t        m_hdrMsgId;   ///< message id (vConnection unique)
  ushort_t        m_hdrBodyLen; ///< message body length
} BsProxyMsgHdr_T;
/*! \} */

#ifndef SWIG
C_DECLS_END
#endif


#endif // _BOTSENSE_H
