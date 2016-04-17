################################################################################
#
# SmartLove.py
#

""" SmartLove Braitenberg Brain Implant.

Authors: Rachel H. Knight
Email:   rachel.knight@roadnarrows.com
Date:    2008.05.25
"""

#
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

import  sys
import random

import Fusion.Hemisson.Cmd.HemiCmdLinCam as HemiLinCam

#-------------------------------------------------------------------------------
# Global Data
#-------------------------------------------------------------------------------

NumPixels     = HemiLinCam.LinCamNumPixels
Brain         = None
CookedPixels  = [0] * NumPixels

#-------------------------------------------------------------------------------
# Standard SmartLove Functions
#-------------------------------------------------------------------------------

#--
def SmartLoveInit(brain):
  """ Initialize the science.

      Parameters:
        brain - optic flow brain primitive

      Return Value:
        None
  """
  global Brain
  Brain = brain

#--
def SmartLoveSetCookedPixels():
  """ Set the brain's processed pixels. """
  global Brain, CookedPixels
  if not Brain:
    return
  id = 'cooked_pixels'
  Brain.mBrum[id] = CookedPixels
  Brain.GuiVizUpdate(id)

#--
def SmartLoveSetSpeeds(speedLeft, speedRight):
  """ Set the robot speeds. """
  global Brain
  if not Brain:
    return
  Brain.mBrum['speed_left']  = speedLeft
  Brain.mBrum['speed_right'] = speedRight

#--
def SmartLoveSay(spokenText, writtenText=None):
  """ Say something. """
  global Brain
  if not Brain:
    return
  Brain.Say('blue', spokenText, writtenText)

#----------------------------------------------------------------------------
# Rachel Knight Stuff
#
# From the brain, you have:
#   Brain.mBrum['cooked_pixels']      - 102 left-to-write pixels
#   Brain.mBrum['speed_left']         - left moter speed -9 to 9
#   Brain.mBrum['speed_right']        - right motor speed -9 to 9
#   Brain.mBrum['bias_left']          - left motor bias 0 to 9
#   Brain.mBrum['bias_right']         - right motor bias 0 to 9
#   Brain.mBrum['nn_vis_threshold']   - pixel threshold
#   Brain.mBrum['nn_vis_scaler']      - visual neurons scaler
#   Brain.mBrum['nn_mem_scaler']      - memory neurons scaler
#   Brain.mBrum['nn_mem_num']         - number of memory neurons
#----------------------------------------------------------------------------

# Rachel: This is your main function. Do all work here
#--
def SmartLoveThinkAct():
  """ Think-Act with the optic flow. """
  global Brain, CookedPixels
  pix = SetThreshold(Brain.mBrum['cooked_pixels'], 0)
  CookedPixels = Brain.mBrum['cooked_pixels']

# Rachel: Some standard helper functions - use as needed.

#--
def StretchContrast(pixlist, minmin=0, maxmax=0xff):
  """ Stretch the current image row to the maximum dynamic range with 
      minmin mapped to black(0x00) and maxmax mapped to white(0xff) and 
      all other pixel values stretched accordingly."""
  if minmin < 0: minmin = 0           # pixel minimum is 0
  if maxmax > 0xff: maxmax =  0xff    # pixel maximum is 255
  if maxmax < minmin: maxmax = minmin # range sanity
  min, max = maxmax, minmin
  for pix in pixlist:
    if pix < min and pix >= minmin:
      min = pix
    if pix > max and pix <= maxmax:
      max = pix
  if min > max: min = max
  if min == max:
    f = 1.0
  else:
    f = 255.0 / (max - min)
  n = 0
  newpixlist= []
  for pix in pixlist:
    if pix < minmin: pix = minmin
    if pix > maxmax: pix = maxmax
    pix = int((pix - min) * f)
    newpixlist.append (pix)
  return newpixlist
      
#--
def SetThreshold(pixlist, th):
  newpixlist= []
  for pix in pixlist:
    if pix < th:
      newpixlist.append(0)    # black
    else:
      newpixlist.append(pix)
  return newpixlist

#--
def FindCenter(pixlist,start):
  while start<len (pixlist):
    if pixlist[start]>0:
      break
    else:
      start=start+1
  if start>=len (pixlist):
    return None
  end=start
  mass=0
  center=0.0
  while end<len (pixlist):
    if pixlist[end]>0:
      mass = mass+pixlist[end]
      center=center+end*pixlist[end]
    else:
      break
    end=end+1
  return start, end-1, mass, center/mass

#--
def RandSay(phraseList):
  n = len(phraseList) - 1
  n = random.randint(0,n)
  SmartLoveSay(phraseList[n][0], phraseList[n][1])
