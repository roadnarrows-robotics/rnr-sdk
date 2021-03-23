################################################################################
#
# _hemiUtils.py
#

""" Hemisson Utilities

Hemisson utility support functions and data.

Author: Robin D. Knight
Email:  robin.knight@roadnarrowsrobotics.com
URL:    http://www.roadnarrowsrobotics.com
Date:   2005.08.16

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
# Functions
#-------------------------------------------------------------------------------

#--
def iniOpen(mode='r'):
  """ Open the hemisson 'ini' file with the given access mode. The 'ini'
      file specified by the 'HEMISSONSTARTUP' environment variable is first
      tried. If this does not succeed, then an attempt is made to open 
      the the file 'HemiDemo/hemisson.ini', then './hemisson.ini'.
    
      Parameters:
        mode - access mode
                 default: 'r' - readonly

      Return Value:
        Returns an opened file object on success, None on failure.
  """
  inilist = ['HemiDemo/hemisson.ini', './hemisson.ini']
  ini = os.getenv('HEMISSONSTARTUP')
  if ini: inilist.insert(0, ini)
  #print("DBG:", inilist)
  for ini in inilist:
    try:
      f = open(ini, mode)
      return f
    except IOError as err:
      pass
  return None


#--
def iniGetPortList(f):
  """ Parse the opened 'ini' file to build a list of available hemisson
      serial ports.
    
      Parameters:
        f - opened 'ini' file object

      Return Value:
        Returns list of ports. List maybe empty.
  """
  portlist = []
  rePortLine = re.compile('[ \t]*port[0-9]*[ \t]*=[ \t]*(\S+)');
  lines = f.readlines()
  for line in lines:
    match = rePortLine.match(line)
    if match:
      portlist += [match.group(1)]
  return portlist

#--
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
  
#--
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

#--
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

#--
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

#--
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

#--
def cvtCsvIntStr(csvstr, width=3):
  """ Convert Comma-Separated-Value integer string into list of integers.
    
      Parameters:
        cvsstr - comma-separated-value string of integers 
                   format: "ddd,ddd,ddd,ddd,..."
        width  - fixed width of each decimal value substrings
                   default: 3

      Return Value:
        Returns list of converted integer values on success. None on error.
  """
  numvals = len(csvstr) / (width + 1) + 1
  dlist = [0] * numvals
  n = 0
  i = 0
  while i < len(csvstr) and n < numvals:
    d = csvstr[i:i+width]
    try:
      dlist[n] = int(d)
    except (SyntaxError, NameError, TypeError, ValueError):
      hlist[n] = 0
    i += (width + 1)
    n += 1
  return dlist
  
