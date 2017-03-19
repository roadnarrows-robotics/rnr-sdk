////////////////////////////////////////////////////////////////////////////////
//
// Package:   CogniBoost
//
// File:      CogniMem.h
//
/*! \file
 *
 *  $LastChangedDate: 2011-11-01 16:05:13 -0600 (Tue, 01 Nov 2011) $
 *  $Rev: 1475 $
 *
 * \brief CogniMem Declarations
 *
 * Defines the CogniMem neural network chip interface.
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

#ifndef _COGNIMEM_H
#define _COGNIMEM_H

#include "rnr/rnrconfig.h"

#ifndef SWIG
C_DECLS_BEGIN
#endif

/*!
 * \defgroup cb_cm_reg CogniMem Registers
 *
 * The CogniBoost exposes the low-level, yet very powerful interface to the
 * CogniMem Neural Network chip by providing pass-through read/write access to
 * the full CogniMem CM1K registers.
 *
 * \par CogniMem Modes:
 * \anchor cm_modes
 * The CogniMem operates in two modes:
 * \termblock
 * \term <b>LR</b> \termdata - learn/run mode. \endterm
 * \term <b>SR</b> \termdata - save/restore mode. \endterm
 * \endtermblock
 * Depending on the CogniMem's mode, the registers have different read/write
 * permissions and different effects on CogniMem's operation.
 *
 * \warning
 * Some features of the CogniMem can render it inoperable in the CogniBoost
 * context (e.g. the Video Recognition Engine). However a (hard) reboot will
 * reset any jammed hardware.
 *
 * \{
 */

/*!
 * \ingroup cb_cm_reg
 * \defgroup cb_cm_limits CogniMem Limits and Sizes
 *
 * \{
 */
#define CM_NEURON_NUMOF         1024  ///< maximum number of neurons/CM
#define CM_NID_MAX_1_CM         (CM_NEURON_NUMOF - 1)
#define CM_NID_MIN              0     ///< minimum neuron id
#define CM_NID_MAX_1_CM         (CM_NEURON_NUMOF - 1)
                                      ///< maximum neuron id (1 CM in chain)
#define CM_NID_MAX_2_CM         (CM_NEURON_NUMOF * 2 - 1)
                                      ///< maximum neuron id (2 CMs in chain)
#define CM_NID_MAX_63_CM        0x0000ffff
                                      ///< maximum neuron id (63 CMs in chain)
#define CM_NID_MAX_MAX          0x00ffffff
                                      ///< \h_ge 64 CMs in chain (24-bit)

#define CM_PATTERN_COMP_NUMOF   256   ///< maximum number pattern components
#define CM_PATTERN_COMP_IDX_MAX (CM_PATTERN_COMP_NUMOF - 1)
                                      ///< maximum pattern component index
                                      //
#define CM_CAT_NUMOF            32768 ///< maximum number of categories
#define CM_CAT_NO_MATCH         0     ///< special "no match" category
#define CM_CAT_MIN_MATCH        1     ///< minimum "matched" category value
#define CM_CAT_MIN              CM_CAT_NO_MATCH
                                      ///< minimum category value
#define CM_CAT_MAX              (CM_CAT_NUMOF - 1)
                                      ///< maximum category value

#define CM_DIST_MIN             0     ///< minimum distance value
#define CM_DIST_MAX             65535 ///< maximum distance value

#define CM_CLASSIFY_CNT_MIN     1     ///< minimum category classification count
#define CM_CLASSIFY_CNT_MAX     8     ///< maximum category classification count

#define CM_CONTEXT_NUMOF        128   ///< maximum number of contexts
#define CM_CONTEXT_MIN          0     ///< minimum context number
#define CM_CONTEXT_MAX          (CM_CONTEXT_NUMOF - 1)
                                      ///< maximum context number

#define CM_BLOCKS_MAX_NUMOF     256   ///< maximum num of ROI primitive blocks
/*! \} */

/*!
 * \ingroup cb_cm_reg
 * \defgroup cb_cm_reg_ncr CogniMem Neuron Context Register
 *
 * The NCR specifies the context and norm used by the identified neuron.
 *
 * \par Fields:
 * \termblock
 * \term context \termdata Neuron context \endterm
 * \term norm \termdata Distance norm. \endterm
 * \term nid \termdata Neuron Identifier. \endterm
 * \endtermblock
 *
 * \par Mode:
 * \ref cm_modes "LR": -\n
 * \ref cm_modes "SR": RW
 *
 * \{
 */
#define CM_REG_NCR_ADDR           0x00    ///< NCR register address
#define CM_REG_NCR_MASK           0xffff  ///< NCR register mask
#define CM_REG_NCR_DFT            0x0000  ///< NCR register power-on default

#define CM_NCR_FLD_CONTEXT_MASK   0x007f  ///< NCR context field mask
#define CM_NCR_FLD_CONTEXT_SHIFT  0       ///< NCR context field shift
#define CM_NCR_FLD_CONTEXT_MIN    0       ///< NCR context minimum field value
#define CM_NCR_FLD_CONTEXT_MAX    (CM_CONTEXT_NUMOF - 1)
                                          ///< NCR context maximum field value

#define CM_NCR_FLD_NORM_MASK      0x0080  ///< NCR norm field mask
#define CM_NCR_FLD_NORM_SHIFT     7       ///< NCR norm field shift
#define CM_NCR_FLD_NORM_L1        0x0000  ///< L<sub>1</sub> norm field value
#define CM_NCR_FLD_NORM_LSUP      0x0080  ///< L<sub>\h_inf</sub> norm field val

// field is always zero unless there are > 64 CM chips in chain
#define CM_NCR_FLD_NID_EX_MASK    0xff00  ///< NCR nid extension field mask
#define CM_NCR_FLD_NID_EX_SHIFT   8       ///< NCR nid extension field shift
#define CM_NCR_FLD_NID_EX_MIN     0       ///< NCR nid ext minimum field value
#define CM_NCR_FLD_NID_EX_MAX     0xff    ///< NCR nid ext maximum field value
/*! \} */

/*!
 * \ingroup cb_cm_reg
 * \defgroup cb_cm_reg_comp CogniMem Neuron Component Register
 *
 * Writes or reads the pattern component byte.
 *
 * \par Mode:
 * \ref cm_modes "LR": W\n
 * \ref cm_modes "SR": RW
 *
 * \{
 */
#define CM_REG_COMP_ADDR      0x01    ///< COMP register address
#define CM_REG_COMP_MASK      0x00ff  ///< COMP register mask
#define CM_REG_COMP_DFT       0x00    ///< COMP register power-on default
/*! \} */

/*!
 * \ingroup cb_cm_reg
 * \defgroup cb_cm_reg_lcomp CogniMem Neuron Last Component Register
 *
 * Writes the last pattern component byte and launches the associative logic.
 *
 * \par Mode:
 * \ref cm_modes "LR": W\n
 * \ref cm_modes "SR": -
 *
 * \{
 */
#define CM_REG_LCOMP_ADDR     0x02    ///< LCOMP register address
#define CM_REG_LCOMP_MASK     0x00ff  ///< LCOMP register mask
#define CM_REG_LCOMP_DFT      0x00    ///< LCOMP regigister power-on default
#define CM_REG_LCOMP_MIN      0       ///< LCOMP register minimum
#define CM_REG_LCOMP_MAX      CM_PATTERN_COMP_IDX_MAX
                                      ///< LCOMP register maximum
/*! \} */

/*!
 * \ingroup cb_cm_reg
 * \defgroup cb_cm_reg_icomp CogniMem Neuron Component Index Register
 *
 * Sets the neuron memory internal index.
 *
 * \note
 * Also known as INDEXCOMP and IINDEX in the documentation.
 *
 * \par Mode:
 * \ref cm_modes "LR": W\n
 * \ref cm_modes "SR": W
 *
 * \{
 */
#define CM_REG_ICOMP_ADDR     0x03    ///< ICOMP register address
#define CM_REG_ICOMP_MASK     0x00ff  ///< ICOMP register mask
#define CM_REG_ICOMP_DFT      0x00    ///< ICOMP register power-on default
#define CM_REG_ICOMP_MIN      0       ///< ICOMP register minimum
#define CM_REG_ICOMP_MAX      CM_PATTERN_COMP_IDX_MAX
                                      ///< ICOMP register maximum
/*! \} */

/*!
 * \ingroup cb_cm_reg
 * \defgroup cb_cm_reg_dist CogniMem Distance Register
 *
 * \par Mode:
 * \ref cm_modes "LR": R\n
 * \ref cm_modes "SR": R
 *
 * \{
 */
#define CM_REG_DIST_ADDR    0x03        ///< DIST register address
#define CM_REG_DIST_MASK    0xffff      ///< DIST register mask
#define CM_REG_DIST_DFT     0xffff      ///< DIST register power-on default
#define CM_REG_DIST_MIN     CM_DIST_MIN ///< DIST register minimum
#define CM_REG_DIST_MAX     CM_DIST_MAX ///< DIST register maximum
/*! \} */

/*!
 * \ingroup cb_cm_reg
 * \defgroup cb_cm_reg_cat CogniMem Category Register
 *
 * Set or get the current category.
 *
 * \par Fields:
 * \termblock
 * \term cat \termdata 15-bit category. \endterm
 * \term degen \termdata Neuron is [not] degenerated. \endterm
 * \endtermblock
 *
 * \par Mode:
 * \ref cm_modes "LR": RW\n
 * \ref cm_modes "SR": RW
 *
 * \{
 */
#define CM_REG_CAT_ADDR         0x04        ///< CAT register address
#define CM_REG_CAT_MASK         0xffff      ///< CAT register mask
#define CM_REG_CAT_DFT          0xffff      ///< CAT register power-on default

#define CM_CAT_FLD_CAT_MASK     0x7fff      ///< CAT category field mask
#define CM_CAT_FLD_CAT_SHIfT    0           ///< CAT category field shift
#define CM_CAT_FLD_CAT_MIN      CM_CAT_MIN  ///< CAT category minimum
#define CM_CAT_FLD_CAT_MAX      CM_CAT_MAX  ///< CAT category maximum

#define CM_CAT_FLD_DEGEN_MASK   0x8000      ///< CAT degenerated field mask
#define CM_CAT_FLD_DEGEN_SHIFT  15          ///< CAT degenerated field shift
#define CM_CAT_FLD_DEGEN_FALSE  0x00        ///< CAT degen. neuron field value
#define CM_CAT_FLD_DEGEN_TRUE   0x01        ///< CAT degen. neuron field value
/*! \} */

/*!
 * \ingroup cb_cm_reg
 * \defgroup cb_cm_reg_aif CogniMem Active Influence Field Register
 *
 * Reads or writes the neuron's active influence field.
 *
 * \par Mode:
 * \ref cm_modes "LR": -\n
 * \ref cm_modes "SR": RW
 *
 * \{
 */
#define CM_REG_AIF_ADDR     0x05        ///< AIF register address
#define CM_REG_AIF_MASK     0xffff      ///< AIF register mask
#define CM_REG_AIF_DFT      0x4000      ///< AIF register power-on default
#define CM_REG_AIF_MIN      CM_DIST_MIN ///< AIF register minimum
#define CM_REG_AIF_MAX      CM_DIST_MIN ///< AIF register maximum
/*! \} */

/*!
 * \ingroup cb_cm_reg
 * \defgroup cb_cm_reg_minif CogniMem Minimum Influence Field Global Register
 *
 * The Minimum Influence Field defines the lower limit for the Active
 * Influence Field for all the neurons. The AIF will not shrink below this
 * limit. The higher the Minimum Influence Field, the bigger the extension of
 * the “Uncertainty” zones in the decision space.
 *
 * \par Mode:
 * \ref cm_modes "LR": RW\n
 * \ref cm_modes "SR": R
 *
 * \{
 */
#define CM_REG_MINIF_ADDR   0x06        ///< MINIF register address
#define CM_REG_MINIF_MASK   0xffff      ///< MINIF register mask
#define CM_REG_MINIF_DFT    0x0002      ///< MINIF register power-on default
#define CM_REG_MINIF_MIN    CM_DIST_MIN ///< MINIF register minimum
#define CM_REG_MINIF_MAX    CM_DIST_MIN ///< MINIF register maximum
/*! \} */

/*!
 * \ingroup cb_cm_reg
 * \defgroup cb_cm_reg_maxif CogniMem Maximum Influence Field Global Register
 *
 * The Maximum Influence Field defines the default value of a newly committed
 * neuron when no other neuron recognized the vector to learn. The higher the
 * Maximum Influence Field, the larger the similarity domains of the neurons
 * and the more liberal the recognition engine.
 *
 * \par Mode:
 * \ref cm_modes "LR": RW\n
 * \ref cm_modes "SR": -
 *
 * \{
 */
#define CM_REG_MAXIF_ADDR   0x07        ///< MAXIF register address
#define CM_REG_MAXIF_MASK   0xffff      ///< MAXIF register mask
#define CM_REG_MAXIF_DFT    0x4000      ///< MAXIF register power-on default
#define CM_REG_MAXIF_MIN    CM_DIST_MIN ///< MAXIF register minimum
#define CM_REG_MAXIF_MAX    CM_DIST_MIN ///< MAXIF register maximum
/*! \} */

/*!
 * \ingroup cb_cm_reg
 * \defgroup cb_cm_reg_nid CogniMem Neuron Identifer Register
 *
 * \note
 * NID's are 24-bits. But unless there are \h_gt 63 CogniMem chips in a chain,
 * this register suffices. Otherwise the upper 8 bits are saved in the NCR.
 *
 * \par Mode:
 * \ref cm_modes "LR": R\n
 * \ref cm_modes "SR": R
 *
 * \{
 */
#define CM_REG_NID_ADDR       0x0a        ///< NID register address
#define CM_REG_NID_MASK       0xffff      ///< NID register mask
#define CM_REG_NID_DFT        0x0000      ///< NID register power-on default
#define CM_REG_NID_MIN        CM_NID_MIN  ///< NID register minimum
#define CM_REG_NID_1_CM_MAX   CM_NID_MAX_1_CM
                                          ///< NID reg. 1 CM in chain maximum
#define CM_REG_NID_MAX        0xffff      ///< NID register maximum
/*! \} */

/*!
 * \ingroup cb_cm_reg
 * \defgroup cb_cm_reg_gcr CogniMem Global Context Register
 *
 * \par Fields:
 * \termblock
 * \term context \termdata Neuron context \endterm
 * \term norm \termdata Distance norm. \endterm
 * \endtermblock
 *
 * \par Mode:
 * \ref cm_modes "LR": RW\n
 * \ref cm_modes "SR": -
 *
 * \{
 */
#define CM_REG_GCR_ADDR           0x0b    ///< GCR address
#define CM_REG_GCR_MASK           0x00ff  ///< GCR mask
#define CM_REG_GCR_DFT            0x0001  ///< GCR power-on default

#define CM_GCR_FLD_CONTEXT_MASK   0x007f  ///< GCR context field mask
#define CM_GCR_FLD_CONTEXT_SHIFT  0       ///< GCR context field shift
#define CM_GCR_FLD_CONTEXT_MIN    0       ///< GCR context minimum field value
#define CM_GCR_FLD_CONTEXT_MAX    (CM_CONTEXT_NUMOF - 1)
                                          ///< GCR context maximum field value

#define CM_GCR_FLD_NORM_MASK      0x0080  ///< GCR norm field mask
#define CM_GCR_FLD_NORM_SHIFT     7       ///< GCR norm field shift
#define CM_GCR_FLD_NORM_L1        0x0000  ///< L<sup>1</sup> norm field value
#define CM_GCR_FLD_NORM_LSUP      0x0080  ///< L<sup>\h_inf</sup> norm field val
/*! \} */

/*!
 * \ingroup cb_cm_reg
 * \defgroup cb_cm_reg_reset_chain CogniMem Reset Chain Global Register
 *
 * Resets the CogniMem chain pointer to the first neuron by writing any
 * value to this register.
 *
 * \par Mode:
 * \ref cm_modes "LR": -\n
 * \ref cm_modes "SR": W
 *
 * \{
 */
#define CM_REG_RESET_CHAIN_ADDR   0x0c  ///< Reset Chain regiser address
/*! \} */

/*!
 * \ingroup cb_cm_reg
 * \defgroup cb_cm_reg_nsr CogniMem Network Status Register
 *
 * This register is updated after each write of the \ref cb_cm_reg_lcomp.
 *
 * \par Fields:
 * \termblock
 * \term model
 *   \termdata Neural network model
 *   \termdata W
 * \endterm
 * \term mode
 *   \termdata Operation mode.
 *   \termdata W
 * \endterm
 * \term ident
 *   \termdata Pattern identified.
 *   \termdata R
 * \endterm
 * \term uncert
 *   \termdata Uncertain identification.
 *   \termdata R
 * \endterm
 * \endtermblock
 *
 * \par Mode:
 * \ref cm_modes "LR": RW\n
 * \ref cm_modes "SR": W
 *
 * \{
 */
#define CM_REG_NSR_ADDR         0x0d          ///< NSR address
#define CM_REG_NSR_MASK         0x003f        ///< NSR mask
#define CM_REG_NSR_DFT          0x0000        ///< NSR power-on default

#define CM_NSR_FLD_MODEL_MASK   0x0020        ///< NSR model field mask
#define CM_NSR_FLD_MODEL_SHIFT  5             ///< NSR model field shift
#define CM_NSR_FLD_MODEL_KNN    0x0020        ///< KNN model
#define CM_NSR_FLD_MODEL_RBF    0x0000        ///< RBF model

#define CM_NSR_FLD_MODE_MASK    0x0010        ///< NSR mode field mask
#define CM_NSR_FLD_MODE_SHIFT   4             ///< NSR mode field shift
#define CM_NSR_FLD_MODE_SR      0x0010        ///< save/restore mode
#define CM_NSR_FLD_MODE_LR      0x0000        ///< learn/run mode

#define CM_NSR_FLD_ID_MASK      0x0008        ///< NSR identified field mask
#define CM_NSR_FLD_ID_SHIFT     3             ///< NSR identified field shift
#define CM_NSR_FLD_ID_TRUE      0x01          ///< identified
#define CM_NSR_FLD_ID_FALSE     0x00          ///< not identified

#define CM_NSR_FLD_UNC_MASK     0x0004        ///< NSR uncertain field mask
#define CM_NSR_FLD_UNC_SHIFT    2             ///< NSR uncertain field shift
#define CM_NSR_FLD_UNC_TRUE     0x01          ///< uncertain
#define CM_NSR_FLD_UNC_FALSE    0x00          ///< not uncertain
/*! \} */

/*!
 * \ingroup cb_cm_reg
 * \defgroup cb_cm_reg_forcnt CogniMem Forget/Count Global Register
 *
 * When read, returns the number of committed neurons.\n
 * When written, causes the CogniMem to forget all of its trained data.
 *
 * \note
 * CogniBoost Non-volatile memory is not effected.
 *
 * \par Mode:
 * \ref cm_modes "LR": RW\n
 * \ref cm_modes "SR": R
 *
 * \{
 */
#define CM_REG_FORGET_ADDR  0x0f        ///< FORGET register address
#define CM_REG_NCOUNT_ADDR  0x0f        ///< NCOUNT register address
#define CM_REG_NCOUNT_MASK  0xffff      ///< NCOUNT mask
#define CM_REG_NCOUNT_FULL  0xffff      ///< network full count
/*! \} */

/*!
 * \ingroup cb_cm_reg
 * \defgroup cb_cm_reg_top CogniMem Top Register
 *
 * Specifies the top pixel coordinate of the ROI.
 *
 * \note
 * Applies to the optional recognition stage of the CogneMem. Not supported
 * by the CogniBoost.
 *
 * \par Mode:
 * \ref cm_modes "LR": RW\n
 *
 * \{
 */
#define CM_REG_TOP_ADDR       0x11        ///< TOP register address
#define CM_REG_TOP_MASK       0xffff      ///< TOP mask
#define CM_REG_TOP_DFT        200         ///< TOP default
/*! \} */

/*!
 * \ingroup cb_cm_reg
 * \defgroup cb_cm_reg_left CogniMem Left Register
 *
 * Specifies the left pixel coordinate of the ROI.
 *
 * \note
 * Applies to the optional recognition stage of the CogneMem. Not supported
 * by the CogniBoost.
 *
 * \par Mode:
 * \ref cm_modes "LR": RW\n
 *
 * \{
 */
#define CM_REG_LEFT_ADDR      0x12        ///< LEFT register address
#define CM_REG_LEFT_MASK      0xffff      ///< LEFT mask
#define CM_REG_LEFT_DFT       120         ///< LEFT default
/*! \} */

/*!
 * \ingroup cb_cm_reg
 * \defgroup cb_cm_reg_width CogniMem Width Register
 *
 * Specifies the width in pixels of the ROI.
 *
 * \note
 * Applies to the optional recognition stage of the CogneMem. Not supported
 * by the CogniBoost.
 *
 * \par Mode:
 * \ref cm_modes "LR": RW\n
 *
 * \{
 */
#define CM_REG_WIDTH_ADDR     0x13        ///< WIDTH register address
#define CM_REG_WIDTH_MASK     0xffff      ///< WIDTH mask
#define CM_REG_WIDTH_DFT      340         ///< WIDTH default
/*! \} */

/*!
 * \ingroup cb_cm_reg
 * \defgroup cb_cm_reg_height CogniMem Height Register
 *
 * Specifies the height in pixels of the ROI.
 *
 * \note
 * Applies to the optional recognition stage of the CogneMem. Not supported
 * by the CogniBoost.
 *
 * \par Mode:
 * \ref cm_modes "LR": RW\n
 *
 * \{
 */
#define CM_REG_HEIGHT_ADDR    0x14        ///< HEIGHT register address
#define CM_REG_HEIGHT_MASK    0xffff      ///< HEIGHT mask
#define CM_REG_HEIGHT_DFT     220         ///< HEIGHT default
/*! \} */

/*!
 * \ingroup cb_cm_reg
 * \defgroup cb_cm_reg_bwidth CogniMem Block Width Register
 *
 * Specifies the primitive block width in pixels.
 *
 * \note
 * Applies to the optional recognition stage of the CogneMem. Not supported
 * by the CogniBoost.
 *
 * \par Mode:
 * \ref cm_modes "LR": RW\n
 *
 * \{
 */
#define CM_REG_BWIDTH_ADDR    0x15        ///< BWIDTH register address
#define CM_REG_BWIDTH_MASK    0xffff      ///< BWIDTH mask
#define CM_REG_BWIDTH_DFT     20          ///< BWIDTH default
/*! \} */

/*!
 * \ingroup cb_cm_reg
 * \defgroup cb_cm_reg_bheight CogniMem Block Height Register
 *
 * Specifies the primitive block height in pixels.
 *
 * \note
 * Applies to the optional recognition stage of the CogneMem. Not supported
 * by the CogniBoost.
 *
 * \par Mode:
 * \ref cm_modes "LR": RW\n
 *
 * \{
 */
#define CM_REG_BHEIGHT_ADDR   0x16        ///< BHEIGHT register address
#define CM_REG_BHEIGHT_MASK   0xffff      ///< BHEIGHT mask
#define CM_REG_BHEIGHT_DFT    20          ///< BHEIGHT default
/*! \} */

/*!
 * \ingroup cb_cm_reg
 * \defgroup cb_cm_reg_rsr CogniMem Recognition Status Register
 *
 * The RSR provides control of the CogniMem video recognition engine
 * and status information of the last recognition cycle.
 *
 * \note
 * Applies to the optional recognition stage of the CogneMem. Not supported
 * by the CogniBoost.
 *
 * \par Fields:
 * \termblock
 * \term reco_active
 *   \termdata High while recognition engine is executing.
 *   \termdata R
 * \endterm
 * \term v_fv
 *   \termdata Copy V_FV signal.
 *   \termdata R
 * \endterm
 * \term uncert
 *   \termdata Uncertain identification.
 *   \termdata R
 * \endterm
 * \term ident
 *   \termdata Pattern identified and classified.
 *   \termdata R
 * \endterm
 * \term out_en
 *   \termdata Category output enabled on DATA bus (N/A for CogniBoost).
 *   \termdata W
 * \endterm
 * \term reco_en
 *   \termdata Enable or disable video recognition engine.
 *   \termdata W
 * \endterm
 * \endtermblock
 *
 * \par Mode:
 * \ref cm_modes "LR": RW\n
 * \ref cm_modes "SR": -
 *
 * \{
 */
#define CM_REG_RSR_ADDR               0x1c    ///< RSR address
#define CM_REG_RSR_MASK               0x003f  ///< RSR mask
#define CM_REG_RSR_DFT                0x0000  ///< RSR power-on default

#define CM_RSR_FLD_RECO_ACTIVE_MASK   0x0020  ///< RSR exec active field mask
#define CM_RSR_FLD_RECO_ACTIVE_SHIFT  5       ///< RSR exec active field shift
#define CM_RSR_FLD_RECO_ACTIVE_TRUE   0x01    ///< recognition being executed
#define CM_RSR_FLD_RECO_ACTIVE_FALSE  0x00    ///< recognition cycle done

#define CM_RSR_FLD_V_FV_MASK          0x0010  ///< RSR v_fv field mask
#define CM_RSR_FLD_V_FV_SHIFT         4       ///< RSR v_fv field shift
#define CM_RSR_FLD_V_FV_TRUE          0x01    ///< f_fv true
#define CM_RSR_FLD_V_FV_FALSE         0x00    ///< f_fv false

#define CM_RSR_FLD_UNC_MASK           0x0008  ///< RSR uncertain field mask
#define CM_RSR_FLD_UNC_SHIFT          3       ///< RSR uncertain field shift
#define CM_RSR_FLD_UNC_TRUE           0x01    ///< last recogn. was uncertain
#define CM_RSR_FLD_UNC_FALSE          0x00    ///< last recogn. was certain

#define CM_RSR_FLD_ID_MASK            0x0004  ///< RSR identified field mask
#define CM_RSR_FLD_ID_SHIFT           2       ///< RSR identified field shift
#define CM_RSR_FLD_ID_TRUE            0x01    ///< identified classification
#define CM_RSR_FLD_ID_FALSE           0x00    ///< unidentified classification

#define CM_RSR_FLD_OUT_EN_MASK        0x0002  ///< RSR output enable field mask
#define CM_RSR_FLD_OUT_EN_SHIFT       1       ///< RSR output enalbe field shift
#define CM_RSR_FLD_OUT_EN_ENABLE      0x01    ///< enable (N/A for CogniBoost)
#define CM_RSR_FLD_OUT_EN_DISABLE     0x00    ///< disable (N/A for CogniBoost)

#define CM_RSR_FLD_RECO_EN_MASK       0x0001  ///< RSR recognition field mask
#define CM_RSR_FLD_RECO_EN_SHIFT      0       ///< RSR recognition field shift
#define CM_RSR_FLD_RECO_EN_ENABLE     0x01    ///< enable recognition engine
#define CM_RSR_FLD_RECO_EN_DISABLE    0x00    ///< disable recognition engine
/*! \} */

/*!
 * \ingroup cb_cm_reg
 * \defgroup cb_cm_reg_rtdist CogniMem Real-Time Distance Register
 *
 * Smallest neuron distance.
 *
 * \note
 * Applies to the optional recognition stage of the CogneMem. Not supported
 * by the CogniBoost.
 *
 * \par Mode:
 * \ref cm_modes "LR": R\n
 * \ref cm_modes "SR": -
 *
 * \{
 */
#define CM_REG_RTDIST_ADDR  0x1d        ///< RTDIST register address
#define CM_REG_RTDIST_MASK  0xffff      ///< RTDIST register mask
#define CM_REG_RTDIST_MIN   CM_DIST_MIN ///< RTDIST register minimum
#define CM_REG_RTDIST_MAX   CM_DIST_MAX ///< RTDIST register maximum
/*! \} */

/*!
 * \ingroup cb_cm_reg
 * \defgroup cb_cm_reg_rtcat Real-Time Category Register
 *
 * First recognized category.
 *
 * \note
 * Applies to the optional recognition stage of the CogneMem. Not supported
 * by the CogniBoost.
 *
 * \par Mode:
 * \ref cm_modes "LR": R\n
 * \ref cm_modes "SR": -
 *
 * \{
 */
#define CM_REG_RTCAT_ADDR   0x1e        ///< RTCAT register address
#define CM_REG_RTCAT_MASK   0xffff      ///< RTCAT register mask
#define CM_REG_RTCAT_MIN    CM_CAT_MIN  ///< RTCAT register minimum
#define CM_REG_RTCAT_MAX    CM_CAT_MAX  ///< RTCAT register maximum
/*! \} */

/*!
 * \ingroup cb_cm_reg
 * \defgroup cb_cm_reg_roiinit CogniMem Reset ROI Register
 *
 * Resets the Region of Interest to default values.
 *
 * \note
 * Applies to the optional recognition stage of the CogneMem. Not supported
 * by the CogniBoost.
 *
 * \par Mode:
 * \ref cm_modes "LR": RW\n
 *
 * \{
 */
#define CM_REG_ROIINIT_ADDR   0x1f        ///< ROIINIT register address
/*! \} */

/*! \} */

#ifndef SWIG
C_DECLS_END
#endif


// ---------------------------------------------------------------------------
// Bit Manipulation Convenience Macros
// ---------------------------------------------------------------------------

/*!
 * \brief Flip the bits.
 *
 * \par Usage:
 * <em>lval</em> = CM_BITS_NOT(<em>bits</em>);\n
 * <tt>expr</tt> <tt>bit-op</tt> CM_BITS_NOT(<em>bits</em>)
 *
 * \param bits  Bit pattern.
 */
#define CM_BITS_NOT(bits)  (~(bits))

/*!
 * \brief Clear bits in field value.
 *
 *  The bit field is defined by the mask. All field bits are set to zero,
 *  leaving all other bits in the value untouched.
 *
 * \par Usage:
 * <em>lval</em> = CM_BITS_CLR(<em>val</em>, <em>mask</em>);
 * <tt>expr</tt> <tt>bit-op</tt> CM_BITS_CLR(<em>val</em>, <em>mask</em>)
 *
 * \param val   Field value.
 * \param mask  Field mask.
 */
#define CM_BITS_CLR(val, mask) ((val) & CM_BITS_NOT(mask))

/*!
 * \brief Get the field value.
 *
 * The bit filed is defined by the shift and mask.
 *
 * \par Usage:
 * <em>lval</em> = CM_BITS_GET(<em>val</em>, <em>mask</em>);
 * <tt>expr</tt> <tt>bit-op</tt> CM_BITS_GET(<em>val</em>, <em>mask</em>)
 *
 * \param val   Field value.
 * \param mask  Field mask.
 */
#define CM_BITS_GET(val, mask)  ((val) & (mask))

/*!
 * \brief Set the field value in register value.
 *
 * The bit field is defined by the mask.
 *
 * \par Usage:
 * <em>lval</em>  = CM_BITS_SET(<em>reg</em>, <em>val</em>, <em>mask</em>);
 *
 * \param reg   Register value.
 * \param val   Field value.
 * \param mask  Field mask.
 */
#define CM_BITS_SET(reg, val, mask) \
  (CM_BITS_GET(val, mask) | CM_BITS_CLR(reg, mask))

/*!
 * \brief Test if any of the bits are set.
 *
 * The bit filed is defined by the mask.
 *
 * \param val   Field value.
 * \param mask  Field mask.
 */
#define CM_BITS_ISSET(val, mask)  (CM_BITS_GET(val, mask)? true: false)

/*!
 * \brief Pack a 16-bit value into the buffer in big-endian order.
 *
 * \param val       Value to pack.
 * \param [out] buf Output byte buffer.
 */
#define CM_BITS_PACK16(val, buf) \
  do \
  { \
    (buf)[0] = (byte_t)(((val) >> 8) & 0xff); \
    (buf)[1] = (byte_t)((val) & 0xff); \
  } while(0)

/*!
 * \brief Unpack a 16-bit value from the buffer packed in big-endian order.
 *
 * \param buf         Input byte buffer.
 * \param [out] val   Unpacked value.
 */
#define CM_BITS_UNPACK16(buf, val) \
  val = (((buf)[0] << 8) & 0xff) | ((buf)[1] & 0xff)


#endif // _COGNIMEM_H
