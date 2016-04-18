////////////////////////////////////////////////////////////////////////////////
//
// Package:   Wound Measurement
//
// Program:   WMTake
//
// File:      econ32.h
//
/*! \file
 *
 * $LastChangedDate: 2013-02-20 10:20:40 -0700 (Wed, 20 Feb 2013) $
 * $Rev: 2697 $
 *
 * \brief e-Con systems ecam32 header file
 *
 * \author Robin Knight (robin.knight@roadnarrows.com)
 * \author Daniel Packard (daniel@roadnarrows.com)
 *
 * \par Copyright:
 * (C) 2011.  CoMedTec
 * (http://www.comedtec.com)
 * \n All Rights Reserved
 */
// Unless otherwise noted, all materials contained are copyrighted and may not
// be used except as provided in these terms and conditions or in the copyright
// notice (documents and software ) or other proprietary notice provided with
// the relevant materials.
//
//
// IN NO EVENT SHALL THE AUTHOR, COMEDTEC, OR ANY MEMBERS/EMPLOYEES/CONTRACTORS
// OF COMEDTEC OR DISTRIBUTORS OF THIS SOFTWARE BE LIABLE TO ANY
// PARTY FOR DIRECT, INDIRECT, SPECIAL, INCIDENTAL, OR CONSEQUENTIAL
// DAMAGES ARISING OUT OF THE USE OF THIS SOFTWARE AND ITS DOCUMENTATION,
// EVEN IF THE AUTHORS OR ANY OF THE ABOVE PARTIES HAVE BEEN ADVISED OF
// THE POSSIBILITY OF SUCH DAMAGE.
//
// THE AUTHORS AND  COMEDTEC SPECIFICALLY DISCLAIM ANY WARRANTIES,
// INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND
// FITNESS FOR A PARTICULAR PURPOSE. THE SOFTWARE PROVIDED HEREUNDER IS ON AN
// "AS IS" BASIS, AND THE AUTHORS AND DISTRIBUTORS HAVE NO OBLIGATION TO
// PROVIDE MAINTENANCE, SUPPORT, UPDATES, ENHANCEMENTS, OR MODIFICATIONS.
//:w
//
////////////////////////////////////////////////////////////////////////////////

#ifndef _ECON_H
#define _ECON_H

#include <linux/videodev.h>
#include <linux/fb.h>
#include <sys/ioctl.h>
#include <malloc.h>
#include <pthread.h>

/*
 * Standard ECON Header file
 */

	#ifdef HAVE_CONFIG_H
		#include <config.h>
	#endif

	#include <stdio.h>
	#include <stdlib.h>
	#include <string.h>
	#include <fcntl.h>
	#include <termios.h>
	#include <unistd.h>

	#include <sys/types.h>
	#include <sys/stat.h>
	#include <sys/time.h>
	#include <sys/ioctl.h>
	#include <sys/mman.h>

/*
 * include custom header file
 */
	#ifdef HAVE_IOCTL_DEF 
		#include "ioctls_def.h"
	#endif

#include "econ_typedef.h"
#include "econ_error.h"
#include "econ_fn_res_prototype.h"

#define ECAM32_MAX_IMAGE_WIDTH  2048
#define ECAM32_MAX_IMAGE_HEIGHT 1536
#define ECAM32_MAX_PATH_NAME 500

#define ECAM32_READ  0
#define ECAM32_WRITE 1
#define ECAM32_QUERY 2

#define ENABLE		0x01
#define DISABLE		0x00

#define ECAM32_TEST_BUFFER_NUM 2

/*!
 *  \brief v4l2 sensor interface
 */
#define V4L2_SENS_TRIG_FOCUS			 (V4L2_CID_PRIVATE_BASE + 1)
#define V4L2_SENS_FCS_OLAY			   (V4L2_CID_PRIVATE_BASE + 2)

#define V4L2_SENS_FLASH				     (V4L2_CID_PRIVATE_BASE + 3)
#define V4L2_SENS_FLASH_LUM			   (V4L2_CID_PRIVATE_BASE + 4)
#define V4L2_SENS_FLASH_TORCH			 (V4L2_CID_PRIVATE_BASE + 5)
#define V4L2_SENS_FLASH_FLASH			 (V4L2_CID_PRIVATE_BASE + 6)
#define V4L2_SENS_FLASH_STROBE		 (V4L2_CID_PRIVATE_BASE + 7)
#define V4L2_SENS_FLASH_FLASH_LUM	 (V4L2_CID_PRIVATE_BASE + 10)
#define V4L2_SENS_FLASH_TORCH_LUM	 (V4L2_CID_PRIVATE_BASE + 11)

#define V4L2_SENS_EFFECTS			     (V4L2_CID_PRIVATE_BASE + 8)
#define V4L2_SENS_FOCUS_DISABLE		 (V4L2_CID_PRIVATE_BASE + 9)

#define V4L2_SENS_ANTISHAKE			   (V4L2_CID_PRIVATE_BASE + 12)
#define V4L2_SENS_ANTISHAKE_STATUS (V4L2_CID_PRIVATE_BASE + 13)

/*! 
 * \brief ecam32 camera control structure
 */
typedef struct _ecam32_camera_data
{
  	INT32 fd_v4l2;

	struct  v4l2_format		fmt;

	struct  v4l2_control	ctrl_whitebalance,ctrl_brightness,			\
					ctrl_contrast,ctrl_saturation,ctrl_exposure,		\
					ctrl_focus,ctrl_sharpness,ctrl_hue,ctrl_effects,	\
					ctrl_h_mirror,ctrl_flash,ctrl_anti_shake,ctrl_v_flip;

	struct  v4l2_queryctrl 	qctrl_whitebalance,qctrl_brightness,qctrl_contrast,	\
					qctrl_saturation,qctrl_exposure,qctrl_focus,		\
					qctrl_sharpness,qctrl_effects,qctrl_v_flip,		\
					qctrl_h_mirror,qctrl_flash,qctrl_anti_shake,qctrl_hue;

	struct v4l2_streamparm		  fps;
	struct v4l2_requestbuffers	req;
	enum   v4l2_buf_type		    type;
	struct v4l2_buffer		      buf;


	PINT8	rgb_888_buffer;
	PINT8	raw_read_buffer;
	UINT8	save_raw_file_needed;

	union
	{
		UINT32 Maintain_threads;
		struct
		{
			UINT32 stream_thread_kill	:1;
		}thread;
	}kill;

	union
	{
		UINT32 G_FLAG;
		struct
		{
			UINT32 comm_ctrl		:2;		// 0 - read, 1 - write, 2 - query 
			UINT32 anti_shake		:1;
		}bit;
	}flag;

	struct __attribute__ ((__packed__))
	{
		UINT16	type; 
		UINT32	size; 
		UINT16	reserved1; 
		UINT16	reserved2; 
		UINT32	offbits; 
		UINT32	size_header; 
		INT32	width; 
		INT32	height; 
		UINT16	planes; 
		UINT16	bitcount; 
		UINT32	compression; 
		UINT32	size_image; 
		INT32	x_permeter; 
		INT32	y_permeter; 
		UINT32	clr_used; 
		UINT32	clr_important; 
	}bmp_header;

	FILE *save_file_ptr;
	INT8 save_path[ECAM32_MAX_PATH_NAME];

	struct fb_var_screeninfo gvinfo;
	struct fb_var_screeninfo vinfo;
	INT32 fb_fd;
	PINT8 fb_ptr;

	INT32 stream_width;
	INT32 stream_height;
	
	pthread_t stream_tid;
	struct testbuffer
	{
		UPINT8 start;
		size_t offset;
		UINT32 length;
	}buffers[ECAM32_TEST_BUFFER_NUM];

	union	
	{
		UINT32 G_FLAG;
		struct
		{
			UINT32 record_mode	:1;
			UINT32 stream_lcd	:1;
		}bit;
	}stream;

	FILE *fp_file_record;

} ecam;

/*!
 *  \brief ecam32 interface
 */
extern FNRESLT register_camera_data(ecam **cam, UINT8 configure_status_flag);
extern FNRESLT save_snap(ecam *cam, const char *sFileName);
extern FNRESLT feature_test_api_init(ecam **cam, int nVidIndex);
extern FNRESLT feature_test_api_exit(ecam *cam);
extern FNRESLT snap_apply_ctrl(ecam *cam);
extern FNRESLT take_snap(ecam *cam);
extern FNRESLT read_snap(ecam *cam);
extern FNRESLT convert_bmp_565_bmp_888(ecam *cam);
extern FNRESLT form_bmp_header_info(ecam *cam);
extern FNRESLT v_flip(ecam *cam);

FNRESLT gettime(OUT PINT64 millisec,UINT8 flag);

#endif // _ECON_H
