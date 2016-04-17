################################################################################
#
# SciPrj.py
#

""" Science Project Module.

Sarah Knight and Amanda Whitney high school science project demonstrating
optic flow algorthms to hunt for 'prey'.

Authors: Sarah J. Knight and Amanda Whitney
Email:   sarah.knight@roadnarrows.com
         awhitney@bak.rr.com
URL:     http://www.thincbam.com
Date:    2006.03.08
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
Blobs         = {}
BoiId         = None
Behavior      = 'hunting'
PrevCenter    = 0
RestingCnt    = 0
SayCnt        = 0

#-------------------------------------------------------------------------------
# Standard SciPrj Functions
#-------------------------------------------------------------------------------

#--
def SciPrjInit(brain):
  """ Initialize the science.

      Parameters:
        brain - optic flow brain primitive

      Return Value:
        None
  """
  global Brain

  Brain = brain
  Behavior = 'hunting'

#--
def SciPrjSetCookedPixels():
  """ Set the brain's processed pixels. """
  global Brain
  if not Brain:
    return
  id = 'cooked_pixels'
  Brain.mBrum[id] = CookedPixels
  Brain.GuiVizUpdate(id)

#--
def SciPrjSetBlobs():
  """ Set the brain's blobs. """
  global Brain, Blobs
  if not Brain:
    return
  id = 'blobs'
  Brain.mBrum[id] = Blobs
  Brain.GuiVizUpdate(id)

#--
def SciPrjSetBoiId():
  """ Set the brain's blob of interest id. """
  global Brain, BoiId
  if not Brain:
    return
  id = 'boi_id'
  Brain.mBrum[id] = BoiId
  Brain.GuiVizUpdate(id)

#--
def SciPrjSetSpeeds(speedLeft, speedRight):
  """ Set the robot speeds. """
  global Brain
  if not Brain:
    return
  Brain.mBrum['speed_left']  = speedLeft
  Brain.mBrum['speed_right'] = speedRight

#--
def SciPrjSay(spokenText, writtenText=None):
  """ Say something. """
  global Brain, Behavior  # RDK 
  if not Brain:
    return
  # RDK added behavior to call so we can have pretty colors
  Brain.Say(Behavior, spokenText, writtenText)


#-------------------------------------------------------------------------------
# Sarah and Amanda Hooks
#-------------------------------------------------------------------------------

#--
def SciPrjThinkAct():
  """ Think-Act with the optic flow. """
  global Brain, CookedPixels
  CookedPixels = Brain.mBrum['cooked_pixels']
  if Behavior == 'hunting':
    hunting()
  elif Behavior == 'chasing':
    chasing()
  else:
    eating()


#-------------------------------------------------------------------------------
# Sarah and Amanda Functions
#-------------------------------------------------------------------------------
def hunting():
    global Brain, CookedPixels, Blobs, BoiId, Behavior, PrevCenter, RestingCnt
    wit=[
        ['i am terribly famished', None],
        ['a hunting i shall go', None],
    ]
    SciPrjSetSpeeds(0,0)
    vis()
    if BoiId == 'prey':
        Behavior = 'chasing'
        PrevCenter = Blobs['prey']['center']
    else:
        RandSay(wit)
        RestingCnt = RestingCnt + 1
        if RestingCnt >= 3:
            leftspeed = random.randint(-9, 9)
            rightspeed = -leftspeed
            RestingCnt = 0
            SciPrjSetSpeeds (leftspeed, rightspeed)
    

def chasing():
    global Brain, CookedPixels, Blobs, BoiId, Behavior, RestingCnt, PrevCenter
    wit=[
        ['fee fi fo fum', None],
        ['you can run but you can not hide', None],
    ]
    leftspeed = rightspeed = Brain.mBrum['hunter_speed']
    if Blobs['prey']['center']<= 45:
        leftspeed = leftspeed -1    # SJK it turns too fast, set to 1
        rightspeed = rightspeed + 1   # SJK
    elif Blobs['prey']['center']>= 55:
        leftspeed = leftspeed + 1   # SJK
        rightspeed = rightspeed -1    # SJK
    dp = Blobs['prey']['center']-PrevCenter
    if dp < -3:
        leftspeed = leftspeed -1
        rightspeed = rightspeed + 1
    elif dp > 3:
        leftspeed = leftspeed + 1
        rightspeed = rightspeed -1
    SciPrjSetSpeeds (leftspeed, rightspeed)
    PrevCenter = Blobs['prey']['center']
    vis()
    if BoiId == 'sadness':
        Behavior = 'hunting'
    # SJK hemi is at the victim if it is large _and_ it straddles the center
    elif Blobs['prey']['end']-Blobs['prey']['start'] > 50 \
        and Blobs['prey']['start'] <= 50  and Blobs['prey']['end'] >= 50:
        SciPrjSetSpeeds(0,0)
        Behavior = 'eating'
    else:
        RandSay(wit)

def eating():
    global Brain, CookedPixels, Blobs, BoiId, Behavior, RestingCnt, PrevCenter
    wit=[
        ['yummy', None],
        ['tastes like chicken', None],
    ]
    vis()
    if BoiId == 'sadness':
        Behavior = 'hunting'
    elif Blobs['prey']['end']-Blobs['prey']['start'] < 35:
        Behavior = 'hunting'
    else:
        RandSay(wit)
    

def vis():
    global Brain, CookedPixels, Blobs, BoiId, Behavior
    #CookedPixels = StretchContrast (CookedPixels) # SJK confusing, dont use
    CookedPixels = threshold(CookedPixels, Brain.mBrum['boi_threshold'])
    SciPrjSetCookedPixels()
    bloblist = findblobs(CookedPixels)
    blobnum = findbestblob(bloblist)
    if blobnum >= 0:
        Blobs["prey"] = {
            'start':bloblist[blobnum][0],
            'end':bloblist[blobnum][1],
            'mass':bloblist[blobnum][2],
            'center':bloblist[blobnum][3],
        }
        BoiId = 'prey'
    else:
        Blobs = {}
        BoiId = 'sadness'
    SciPrjSetBlobs()
    SciPrjSetBoiId()
    
          

def StretchContrast(pixlist, minmin=0, maxmax=0xff):
    """Stretch the current image row to the maximum dynamic range with 
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
      
def threshold (pixlist, th):
    newpixlist= []
    for pix in pixlist:
        if pix < th:
            newpixlist.append (0)
        else:
            newpixlist.append (pix)
    return newpixlist


def findcenter (pixlist,start):
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

def findblobs (pixlist):
    start=0
    bloblist=[]
    while True:
        data=findcenter (pixlist,start)
        if not data:
            return bloblist
        bloblist.append(data)
        start, end, mass, center = data
        #print start, end, mass, center
        start=end+1

def findbestblob(blobs):
    maxmass = 0
    n = 0
    blobnum = -1
    for blob in blobs:
        if blob[2] > maxmass:
            blobnum = n
            maxmass = blob[2]
        n = n+1
    return blobnum

def RandSay (wit):
    global SayCnt
    SayCnt = SayCnt + 1
    if SayCnt % 2 != 0:
        return 
    n = len(wit) - 1
    n = random.randint(0,n)
    SciPrjSay(wit[n][0],wit [n][1])


            



