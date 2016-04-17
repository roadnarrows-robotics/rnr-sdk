////////////////////////////////////////////////////////////////////////////////
//
// Package:   Eudoxus
//
// Program:   euStitch
//
// File:      euStitch.cxx
//
/*! \file
 *
 * $LastChangedDate: 2016-01-12 14:08:01 -0700 (Tue, 12 Jan 2016) $
 * $Rev: 4256 $
 *
 * \brief Eudoxus Point Cloud stitching application.
 *
 * \author Robin Knight (robin.knight@roadnarrows.com)
 *
 * \par Copyright:
 * (C) 2012-2015.  RoadNarrows
 * (http://www.roadnarrows.com)
 * \n All Rights Reserved
 */
/*
 * @EulaBegin@
// Unless otherwise noted, all materials contained are copyrighted and may not
// be used except as provided in these terms and conditions or in the copyright
// notice (documents and software ) or other proprietary notice provided with
// the relevant materials.
//
//
// IN NO EVENT SHALL THE AUTHOR, ROADNARROWS, OR ANY MEMBERS/EMPLOYEES/
// CONTRACTORS OF ROADNARROWS OR DISTRIBUTORS OF THIS SOFTWARE BE LIABLE TO ANY
// PARTY FOR DIRECT, INDIRECT, SPECIAL, INCIDENTAL, OR CONSEQUENTIAL
// DAMAGES ARISING OUT OF THE USE OF THIS SOFTWARE AND ITS DOCUMENTATION,
// EVEN IF THE AUTHORS OR ANY OF THE ABOVE PARTIES HAVE BEEN ADVISED OF
// THE POSSIBILITY OF SUCH DAMAGE.
//
// THE AUTHORS AND  ROADNARROWS SPECIFICALLY DISCLAIM ANY WARRANTIES,
// INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND
// FITNESS FOR A PARTICULAR PURPOSE. THE SOFTWARE PROVIDED HEREUNDER IS ON AN
// "AS IS" BASIS, AND THE AUTHORS AND DISTRIBUTORS HAVE NO OBLIGATION TO
// PROVIDE MAINTENANCE, SUPPORT, UPDATES, ENHANCEMENTS, OR MODIFICATIONS.
//
 * @EulaEnd@
 */
////////////////////////////////////////////////////////////////////////////////

#include "Eudoxus/euConf.h"

#include <sys/select.h>
#include <stdio.h>
#include <cfloat>

#include <Eigen/Geometry>
#include <flann/flann.h>

#include <pcl/common/common.h>
#include <pcl/io/pcd_io.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/icp_nl.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/point_cloud_handlers.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/histogram_visualizer.h>
#include <pcl/visualization/point_picking_event.h>
#include <pcl/console/print.h>
#include <pcl/console/parse.h>
#include <pcl/console/time.h>

#include <vtkPolyDataReader.h>

#include <espeak/speak_lib.h>

#include "rnr/rnrconfig.h"
#include "rnr/log.h"
#include "rnr/opts.h"

#include "Eudoxus/Eudoxus.h"
#include "Eudoxus/euPcd.h"
#include "Eudoxus/euClientUdp.h"

#include "version.h"

using namespace pcl;
using namespace eu;

#define APP_EC_OK     0   ///< success exit code
#define APP_EC_ARGS   2   ///< command-line options/arguments error exit code
#define APP_EC_EXEC   4   ///< execution exit code

//
// Command-Line Options
//
static char  *Argv0;                                    ///< the command
static int    OptsPort        = 4000;                   ///< UDP port
static double OptsMinZ        = 100.0;                  ///< minimum z (mm)
static double OptsMaxZ        = 10000.0;                ///< maximum z (mm)
static int    OptsDelaySec    = 0;                      ///< delay before map
static int    OptsNumFrames   = 0;                      ///< number map frames
static char  *OptsOutFileName = (char *)"./3dmap.pcd";  ///< output file name
static bool_t OptsShowConv    = false;                  ///< show convergence
static bool_t OptsAudio       = false;                  ///< use audio prompts

//
// Forward Declarations
//
static int OptsCvtArgTopology(const char *argv0, const char *sOptName,
                              char *optarg, void *pOptVal);
/*!
 * \brief Program information.
 */
static OptsPgmInfo_T AppPgmInfo =
{
  // usage_args
  NULL,

  // synopsis
  "The %P application stitches point cloud data.",

  // long_desc = 
  "...",
 
  // diagnostics
  NULL
};

/*!
 * \brief Command line options information.
 */
static OptsInfo_T AppOptsInfo[] =
{
  // --port, -p
  {
    "port",               // long_opt
    'p',                  // short_opt
    required_argument,    // has_arg
    true,                 // has_default
    &OptsPort,            // opt_addr
    OptsCvtArgInt,        // fn_cvt
    OptsFmtInt,           // fn_fmt
    "<number>",           // arg_name
    "UDP port number."    // opt desc
  },

  // --z-min
  {
    "z-min",              // long_opt
    OPTS_NO_SHORT,        // short_opt
    required_argument,    // has_arg
    true,                 // has_default
    &OptsMinZ,            // opt_addr
    OptsCvtArgFloat,      // fn_cvt
    OptsFmtFloat,         // fn_fmt
    "<mm>",               // arg_name
    "Minimum depth. Less than z-min values are cropped." // opt desc
  },

  // --z-max
  {
    "z-max",              // long_opt
    OPTS_NO_SHORT,        // short_opt
    required_argument,    // has_arg
    true,                 // has_default
    &OptsMaxZ,            // opt_addr
    OptsCvtArgFloat,      // fn_cvt
    OptsFmtFloat,         // fn_fmt
    "<mm>",               // arg_name
    "Maximum depth. Greater than z-max values are cropped."  // opt desc
  },

  // --delay
  {
    "delay",              // long_opt
    OPTS_NO_SHORT,        // short_opt
    required_argument,    // has_arg
    true,                 // has_default
    &OptsDelaySec,        // opt_addr
    OptsCvtArgInt,        // fn_cvt
    OptsFmtInt,           // fn_fmt
    "<seconds>",          // arg_name
    "Delay time (seconds) until mapping is started." // opt desc
  },

  // --num-frames
  {
    "num-frames",         // long_opt
    OPTS_NO_SHORT,        // short_opt
    required_argument,    // has_arg
    true,                 // has_default
    &OptsNumFrames,       // opt_addr
    OptsCvtArgInt,        // fn_cvt
    OptsFmtInt,           // fn_fmt
    "<count>",            // arg_name
    "Maximum number of frames to stitch to build map. "
    "If zero, then user terminated."
  },

  // --file, -f
  {
    "file",               // long_opt
    'f',                  // short_opt
    required_argument,    // has_arg
    true,                 // has_default
    &OptsOutFileName,     // opt_addr
    OptsCvtArgStr,        // fn_cvt
    OptsFmtStr,           // fn_fmt
    "<filename>",         // arg_name
    "Output file name path."  // opt desc
  },

  // --show-convergence
  {
    "show-convergence",   // long_opt
    OPTS_NO_SHORT,        // short_opt
    no_argument,          // has_arg
    true,                 // has_default
    &OptsShowConv,        // opt_addr
    OptsCvtArgBool,       // fn_cvt
    OptsFmtBool,          // fn_fmt
    NULL,                 // arg_name
    "Do [not] show convergence."  // opt desc
  },

  // --audio
  {
    "audio",              // long_opt
    'a',                  // short_opt
    no_argument,          // has_arg
    true,                 // has_default
    &OptsAudio,           // opt_addr
    OptsCvtArgBool,       // fn_cvt
    OptsFmtBool,          // fn_fmt
    NULL,                 // arg_name
    "Do [not] audio prompts."  // opt desc
  },

  {NULL, }
};

typedef struct
{
  float m_fLeafSize;
  float m_fMaxCorrDistStart;
  float m_fMaxCorrDistEnd;
  float m_fDistStepSize;
  int   m_iter;
} ConvProf_T;

int First = 1;


#ifdef RDK
//Making a cloud to display each step through
PointCloud<PointXYZRGB>::Ptr ShoCloud(new PointCloud<PointXYZRGB>);

// Creating a downsampling object 'Down'
VoxelGrid<PointXYZRGB> Down;
#endif // RDK

// Creating an iterative closest points object 'ClosePoints'
IterativeClosestPoint<PointXYZRGB, PointXYZRGB> ClosePoints;

#ifdef RDK
// Creating a transformation matrix for all combined transformations
Eigen::Matrix4f TotalTransformation;
#endif // RDK

/*!
 * \brief Block, waiting for either timeout or user keyboard press.
 *
 * No characters are read.
 *
 * \param nMSec   Wait period (ms)
 *
 * \return Returns true if wait was interrupted by keyboard press. Else returns
 * false on time out.
 */
static bool waitkey(int nMSec)
{
   int            fno = fileno(stdin);
   fd_set         fdset;
   uint_t         usec = (uint_t)nMSec * 1000;
   struct timeval timeout;
   int            rc;

   FD_ZERO(&fdset);
   FD_SET(fno, &fdset);
   
   // timeout (gets munged after each select())
   timeout.tv_sec  = (time_t)(usec / 1000000);
   timeout.tv_usec = (time_t)(usec % 1000000);

   if( (rc = select(fno+1, &fdset, NULL, NULL, &timeout)) > 0 )
   {
     fflush(stdin);
   }

   return rc>0? true: false;
}

static void prompt(const char *s)
{
	static int synth_flags = espeakCHARS_AUTO | espeakPHONEMES | espeakENDPAUSE;
  
  if( OptsAudio )
  {
    espeak_Synth(s, strlen(s)+1, 0, POS_CHARACTER, 0, synth_flags, NULL, NULL);
  }

  printf("%s", s);
  fflush(stdout);
}

//Function to do stitching
int Stitch(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &targetCloud, 
           pcl::PointCloud<pcl::PointXYZRGB>::Ptr &transformingCloud,
           float                                   leafSize,
           float                                   maxDistance,
           int                                     iterations,
           pcl::visualization::CloudViewer        &viewer)
{
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr targCloud 
                                   (new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr transCloud 
                                   (new pcl::PointCloud<pcl::PointXYZRGB>);
  Eigen::Matrix4f transformation;

  // setting the leaf size to 'leafSize'
  //RDK Down.setLeafSize(leafSize, leafSize, leafSize);

  // setting input to 'TargetCloud'
  //RDK Down.setInputCloud(targetCloud);

  // starting downsampling and placing output in 'targCloud'
  //RDK Down.filter(*targCloud);

  // setting input to 'TransformingCloud'
  //RDK Down.setInputCloud(transformingCloud);

  // starting downsampling and placing output in 'transCloud'
  //RDK Down.filter(*transCloud);

  // setting the grounded cloud as 'targCloud'
  ClosePoints.setInputTarget(targCloud);

  // setting the cloud to be aligned as 'transCloud'
  ClosePoints.setInputCloud(transCloud);

  // setting the max number of iterations before giving up to 15
  ClosePoints.setMaximumIterations(15); 

  // setting distance for points to be considered coresponding to 'maxDistance'
  ClosePoints.setMaxCorrespondenceDistance(maxDistance);

  // setting Threshhold for RANSAC to .5
  ClosePoints.setRANSACOutlierRejectionThreshold((OptsMaxZ-OptsMinZ)/100.0);

  for(int i=0; i<iterations; ++i)
  {
    ClosePoints.align(*transCloud);

    if( ClosePoints.hasConverged() )
    {
      if(First)
      {
        transformation = ClosePoints.getFinalTransformation();
        //RDK TotalTransformation = transformation;
        First = 0;
      }

      else
      {
        transformation = ClosePoints.getFinalTransformation ();
        //RDK TotalTransformation = (TotalTransformation * transformation);
      }

      pcl::transformPointCloud(*transformingCloud, *transformingCloud,
                                                             transformation);

      if( OptsShowConv )
      {
        //RDK pcl::copyPointCloud(*transformingCloud, *ShoCloud);
        //RDK *ShoCloud += *targetCloud;
        //RDK viewer.showCloud(ShoCloud);
      }

      return 0;
    }
  }

  return -1;
}

void FilterPT(PointCloud<PointXYZRGB>::Ptr &shrinkCloud)
{
  PassThrough<PointXYZRGB> pass_through;

  pass_through.setFilterFieldName("z");
  pass_through.setFilterLimits(OptsMinZ, OptsMaxZ);
  pass_through.setInputCloud(shrinkCloud);
  pass_through.filter(*shrinkCloud);
}

/*!
 * \brief Main command-line argument initialization.
 *
 * \param argc    Command-line argument count.
 * \param argv    Command-line argument list.
 *
 * \par Exits:
 * Program terminates on conversion error.
 */
static void MainInitArgs(int argc, char *argv[])
{
 // name of this process
  Argv0 = basename(argv[0]);

  // parse input options
  argv = OptsGet(Argv0, &PkgInfo, &AppPgmInfo, AppOptsInfo, true, &argc, argv);
}

/*!
 * \brief Main initialization.
 *
 * \param argc    Command-line argument count.
 * \param argv    Command-line argument list.
 *
 * \par Exits:
 * Program terminates on conversion error.
 */
static void MainInit(int argc, char *argv[])
{
  int   sampleRate;

  // parse, validate, and save command-line arguments
  MainInitArgs(argc, argv);

  if( OptsAudio )
  {
    sampleRate = espeak_Initialize(AUDIO_OUTPUT_PLAYBACK, 0, NULL, 0);

    if( sampleRate < 0 )
    {
      LOGERROR("espeak_Initialize(): Failed to initialize - "
               "turning off audio.\n");
      OptsAudio = false;
    }
  }
}

ConvProf_T ConvProfile[] =
{
  {150.0,   400.0,    200.0,    20.0,   10},
  {100.0,   180.0,    100.0,    10.0,   10},
  {50.0,     90.0,     50.0,    10.0,   10},
  {25.0,     00.0,     10.0,    10.0,   10}
};

size_t NumConvProfEntries = arraysize(ConvProfile);

int Align(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &primeCloud, 
          pcl::PointCloud<pcl::PointXYZRGB>::Ptr &secondaryCloud,
          pcl::visualization::CloudViewer        &viewer)
{
  size_t  i;
  float   coresp;

  for(i=0; i<NumConvProfEntries; ++i)
  {
    printf(" leaf_size=%5.1f max_corr_dist=", ConvProfile[i].m_fLeafSize);

    for(coresp  = ConvProfile[i].m_fMaxCorrDistStart;
        coresp >= ConvProfile[i].m_fMaxCorrDistEnd;
        coresp -= ConvProfile[i].m_fDistStepSize)
    {
      printf("%5.1f ", coresp); fflush(stdout);

      if( Stitch(primeCloud, secondaryCloud, ConvProfile[i].m_fLeafSize,
                    coresp, ConvProfile[i].m_iter, viewer) < 0 )
      {
        printf("\nFailed to align new frame - skipping.\n");
        return -1;
      }
    }
    printf("\n");
  }

  return 0;
}

int main(int argc, char* argv[])
{
  fprintf(stderr, "main()\n");
  PCLPointCloud2                  cloud;
  PointCloud<PointXYZRGB>::Ptr    primeCloud(new PointCloud<PointXYZRGB>);
  PointCloud<PointXYZRGB>::Ptr    secondaryCloud(new PointCloud<PointXYZRGB>);
  Eigen::Vector4f                 origin;
  Eigen::Quaternionf              orientation;
  int                             frameNumber;
  char                            buf[256];
  int                             i;

  MainInit(argc, argv);

  OniGstClientUdp grabber(OptsPort, 4);
  
  grabber.start();

  if( OptsDelaySec > 0 )
  {
    printf("Mapping starting in %d seconds.\n", OptsDelaySec);
    printf("Press <CR> to stop mapping.\n\n");
    fflush(stdout);

    // count down
    for(i=0; i<OptsDelaySec; ++i)
    {
      sprintf(buf, "%d ", OptsDelaySec-i);
      prompt(buf);
      sleep(1);
    }

    prompt("Go\n\n");
  }

  printf("Grabbing starting frame ... "); fflush(stdout);

  grabber.grabPointCloudFrame(cloud, origin, orientation);

  fromPCLPointCloud2(cloud, *primeCloud);

  printf("grabbed.\n"); fflush(stdout);

  frameNumber = 0;

  FilterPT(primeCloud);

  //RDK copyPointCloud(*primeCloud, *ShoCloud);

  // create visualizer
  pcl::visualization::CloudViewer viewer("Map Viewer");

  //RDK viewer.showCloud(ShoCloud);

  while(((OptsNumFrames == 0) || (frameNumber < OptsNumFrames)) && !waitkey(1))
  {
    printf("Grabbing new frame ... "); fflush(stdout);

    grabber.grabPointCloudFrame(cloud, origin, orientation);

    fromPCLPointCloud2(cloud, *secondaryCloud);

    printf("grabbed.\n"); fflush(stdout);

    FilterPT(secondaryCloud);

    if( !First )
    {
      //RDK pcl::transformPointCloud(*secondaryCloud, *secondaryCloud,
      //RDK                                              TotalTransformation);
    }

    printf("Stitching point clouds ... \n"); fflush(stdout);

    if( Align(primeCloud, secondaryCloud, viewer) != -1 )
    {
      printf("Stitched.\n"); fflush(stdout);

      *primeCloud += *secondaryCloud;

      //RDK pcl::copyPointCloud(*primeCloud, *ShoCloud);

      //RDK viewer.showCloud(ShoCloud);

      ++frameNumber;

      printf("Added %d frames to 3D map.\n", frameNumber); fflush(stdout);
    }
  }

  grabber.stop();

  printf("Saving 3D map to %s ... ", OptsOutFileName); fflush(stdout);

  io::savePCDFileBinary(OptsOutFileName, *primeCloud);

  printf("saved.\n"); fflush(stdout);

  return 0;
}
