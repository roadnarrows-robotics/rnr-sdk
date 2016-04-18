###############################################################################
#
# Module:   Laelaps.Utils
#
# Package:  RoadNarrows Laelaps Robotic Mobile Platform Package
#
# Link:     https://github.com/roadnarrows-robotics/laelaps
#
# File:     Utils.py
#
## \file 
##
## $LastChangedDate: 2016-03-17 12:14:21 -0600 (Thu, 17 Mar 2016) $
## $Rev: 4350 $
##
## \brief Utilities.
##
## \author Robin Knight (robin.knight@roadnarrows.com)
##  
## \par Copyright:
##   (C) 2015-2016.  RoadNarrows LLC.\n
##   (http://www.roadnarrows.com)\n
##   All Rights Reserved
##
# @EulaBegin@
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
# @EulaEnd@
#
###############################################################################

import sys
import os
import time
import math

from pkg_resources import *

from Tkinter import *
from Tkconstants import *
from tkFileDialog import *
import tkFont

from PIL import Image, ImageTk

# ------------------------------------------------------------------------------
# Class ImageLoader
# ------------------------------------------------------------------------------

#
## \brief Class to handle image loading.
#
class ImageLoader:
  #
  ## \brief Constructor
  ##
  ## \param py_pkg      Python resource (e.g. "laelaps_control.images").
  ## \param image_paths List of directory paths to search for the image.
  #
  def __init__(self, py_pkg=None, image_paths=[]):
    self.m_pyPkg = py_pkg
    if len(image_paths) > 0:
      self.m_imagePaths = image_paths
    else:
      self.m_imagePaths = ['.']
  
  #
  ## \brief Open image from file and convert to PhotoImage.
  ##
  ## \param filename    Image file name.
  ##
  ## \return Returns image widget on success, None on failure.
  #
  def loadImage(self, filename):
    img = self.openImage(filename)
    if img is not None:
      return ImageTk.PhotoImage(img)
    else:
      return None

  """
    # no file name
    if filename is None or len(filename) == 0:
      return None;
    # absolute file name
    if filename[0] == os.path.sep:
      try:
        return ImageTk.PhotoImage(Image.open(filename))
      except IOError:
        return None
    # relative file name - try python resource(s) first
    if self.m_pyPkg:
      try:
        fqname = resource_filename(self.m_pyPkg, filename)
        try:
          return ImageTk.PhotoImage(Image.open(fqname))
        except IOError:
          pass
      except ImportError:
        pass
    # relative file name - search path for file
    for path in self.m_imagePaths:
      fqname = path + os.path.sep + filename
      try:
        return ImageTk.PhotoImage(Image.open(fqname))
      except IOError:
        continue
    return None
  """

  #
  ## \brief Open image from file.
  ##
  ## \param filename    Image file name.
  ##
  ## \return Returns image widget on success, None on failure.
  #
  def openImage(self, filename):
    # no file name
    if filename is None or len(filename) == 0:
      return None;
    # relative file name - try python resource(s) first
    if self.m_pyPkg:
      try:
        fqname = resource_filename(self.m_pyPkg, filename)
        try:
          return Image.open(fqname)
        except IOError:
          pass
      except ImportError:
        pass
    # relative file name - search path for file
    for path in self.m_imagePaths:
      fqname = path + os.path.sep + filename
      try:
        return Image.open(fqname)
      except IOError:
        continue
    return None



# ------------------------------------------------------------------------------
# Misc. Utilities
# ------------------------------------------------------------------------------

#
#
## Round to nearest 100th.
#
def round100th(x):
  return math.floor((x + 0.005) * 100.0) / 100.0

#
## Round to nearest 10th.
#
def round10th(x):
  return math.floor((x + 0.05) * 10.0) / 10.0

#
## Degrees to radians.
#
def degToRad(deg):
  return deg / 180.0 * math.pi

#
## Radians to degrees.
#
def radToDeg(rad):
  return rad / math.pi * 180.0

#
## Connected Laelaps
#
def whichROSMaster():
  rosmaster = os.getenv('ROS_MASTER_URI')
  if len(rosmaster) == 0:
    return 'localhost'
  parts = rosmaster.split(':')
  if len(parts) < 2:
    return 'localhost'
  host = parts[1]
  return host.strip('/')
