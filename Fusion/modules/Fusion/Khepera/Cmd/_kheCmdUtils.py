################################################################################
#
# _kheUtils.py
#

""" Khepera Utilities

Khepera utility support functions and data.

Author: Robin D. Knight
Email:  robin.knight@roadnarrowsrobotics.com
URL:    http://www.roadnarrowsrobotics.com
Date:   2005.10.31

Copyright (C) 2005, 2006.  RoadNarrows LLC.
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

import  os
import  re


#-------------------------------------------------------------------------------
# Public Interface
#-------------------------------------------------------------------------------

#
# Functions
#

# Cap value between [min, max]
def cap(val, min, max):
  """ Cap the value between the minimum and maximum values, inclusive.
    
      Parameters:
        val - value to cap
        min - minimum allowed value
        max - maximum allowed value

      Return Value:
        Returns capped value.
  """
  if val < min: return min
  if val > max: return max
  return val
  
def cvtHH(hhstr, pos=0):
  """ Convert HH hex [sub]string value into integer.
    
      Parameters:
        hhstr - HH [sub]string
                  format: "....HH...."
        pos   - starting position in string.
                  default: 0 (start of string)

      Return Value:
        Returns converted integer value on success. None on error.
  """
  h = '0x' + hhstr[pos] + hhstr[pos+1]
  try:
    val = eval(h)
    return val
  except (SyntaxError, NameError, TypeError):
    return None

def cvtHHHH(hhhhstr, pos=0):
  """ Convert HHHH hex [sub]string value into integer.
    
      Parameters:
        hhhhstr - HHHH [sub]string
                    format: "....HHHH...."
        pos     - starting position in string.
                    default: 0 (start of string)

      Return Value:
        Returns converted integer value on success. None on error.
  """
  h = '0x' + hhhhstr[pos:pos+4]
  try:
    val = eval(h)
    return val
  except (SyntaxError, NameError, TypeError):
    return None

def cvtCatHHStr(hhstr, numvals):
  """ Convert concatenated HH hex string into list of integers.
    
      Parameters:
        hhstr    - concatenated HH string
                     format: "HHHHHHHH..."
        numvals  - number of HH values in string to convert

      Return Value:
        Returns list of converted integer values on success. None on error.
  """
  dlist = [0] * numvals
  n = 0
  i = 0
  while i < len(hhstr) and n < numvals:
    h = '0x' + hhstr[i:i+2]
    try:
      dlist[n] = eval(h)
    except (SyntaxError, NameError, TypeError):
      dlist[n] = 0
    i += 2
    n += 1
  return dlist

def cvtInt(intstr):
  """ Convert integer string into integer.
    
      Parameters:
        intstr - integer string
                   format: "[0-9]+"

      Return Value:
        Returns converted integer value on success. None on error.
  """
  try:
    val = int(intstr)
    return val
  except (SyntaxError, NameError, TypeError, ValueError):
    return None

def cvtCsvIntStr(csvstr):
  """ Convert Comma-Separated-Value integer string into list of integers.
    
      Parameters:
        cvsstr - comma-separated-value string of integers 
                   format: "d,d,d,d,..."

      Return Value:
        Returns list of converted integer values on success. 
        Returns empty list on error.
  """
  sValList = csvstr.split(',')
  dlist = []
  for sVal in sValList:
    try:
      dlist += [int(sVal)]
    except (SyntaxError, NameError, TypeError, ValueError):
      return []
  return dlist
