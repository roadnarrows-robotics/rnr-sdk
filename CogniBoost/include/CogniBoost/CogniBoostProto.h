////////////////////////////////////////////////////////////////////////////////
//
// Package:   CogniBoost
//
// File:      CogniBoostProto.h
//
/*! \file
 *
 *  $LastChangedDate: 2011-10-19 15:05:47 -0600 (Wed, 19 Oct 2011) $
 *  $Rev: 1398 $
 *
 * \brief CogniBoost Message Protocol Interface Declarations.
 *
 * The protocol defines the packet message interface between the CogniBoost
 * host client and the CogniBoost device server.
 *
 * \author Brent Wilkins  (brent@roadnarrows.com)
 * \author Daniel Packard (daniel@roadnarrows.com)
 * \author Robin Knight   (robin@roadnarrows.com)
 *
 * \par Copyright
 *   \h_copy 2011-2017. RoadNarrows LLC.\n
 *   http://www.roadnarrows.com\n
 *   All Rights Reserved
 *
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
///////////////////////////////////////////////////////////////////////////////

#ifndef _COGNIBOOSTPROTO_H
#define _COGNIBOOSTPROTO_H

#include "rnr/rnrconfig.h"
#include "rnr/netmsgs.h"

#ifndef SWIG
C_DECLS_BEGIN
#endif

/*!
 * \ingroup cb_proto
 * \defgroup cb_proto_msg  CogniBoost Message Attributes
 *
 *
 * \{
 */
#define CB_MSG_ENCODING   NMEncodingITV     ///< id-type-value encoding
#define CB_MSG_ENDIAN     NMEndianBig       ///< big-endian byte order
#define CB_MSG_MIN_SIZE   NMITV_MSGHDR_SIZE
                                  ///< message minumum length, no body (bytes) 
#define CB_MSG_MAX_SIZE   512     ///< message maximum length (bytes)
/*! \} */

/*!
 * \ingroup cb_proto
 * \defgroup cb_proto_pkt  CogniBoost Packet Attributes
 *
 * All \h_cogniboost server-directed requests and client-directed
 * responses contain a fixed-format packet header plus an ITV encoded message. 
 * ITV messages contain the unique message id plus the number of fields (if
 * any) in the message body.
 *
 * \{
 */
#define CB_PKT_HDR_SIZE     4     ///< packet header length (bytes)
#define CB_PKT_MIN_SIZE     (CB_PKT_HDR_SIZE+CB_MSG_MIN_SIZE)
                                  ///< packet minium length (bytes)
#define CB_PKT_MAX_SIZE     (CB_PKT_HDR_SIZE+CB_MSG_MAX_SIZE)
                                  ///< packet maximum length (bytes)

/*!
 * The magic pattern helps the receiver to identify the start of a packet.
 */
#define CB_HDR_MAGIC        0xcb  ///< message header magic pattern
  
/*!
 * \ingroup cb_proto_pkt
 * \defgroup cb_proto_tid  CogniBoost Transaction Id Attributes
 *
 * A transaction id provides an identifier to map responses to requests. Since
 * a \h_cogniboost client can be mulit-threaded, responses can be received by
 * the client out of order. The \h_cogniboost client library automatically
 * reorders the responses and delivers the correct response to the given
 * request.
 *
 * \{
 */
#define CB_HDR_TID_MIN      0     ///< minimum transaction id
#define CB_HDR_TID_MAX      255   ///< maximum transaction id
#define CB_HDR_TID_NUMOF    256   ///< number of unique tid's
#define CB_HDR_TID_MASK     0xff  ///< transaction id mask

typedef uint_t cbTid_T;           /// client transaction id type [0-255]
/*! \} */

/*!
 * \brief CogniBoost Packet Header Structure
 */
typedef struct
{
  byte_t          m_hdrMagic;   ///< "unique" magic pattern
  byte_t          m_hdrTid;     ///< transaction id
  ushort_t        m_hdrMsgLen;  ///< message length (bytes)
} cbPktHdr_T;

/*! \} */


#ifndef SWIG
C_DECLS_END
#endif

#endif // _COGNIBOOSTPROTO_H
