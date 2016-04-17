################################################################################
#
# GuiTypes.py
#

""" Graphical User Interface Types Module

Implementation independent Graphical User Interface (GUI) data
types and functions module.

Although the data types may match the Tkinter.Menu types, only the
concepts are copied. 

This module should remain independent of any specific GUI implementation.


Author: Robin D. Knight
Email:  robin.knight@roadnarrowsrobotics.com
URL:    http://www.roadnarrowsrobotics.com
Date:   2005.12.04

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

#-------------------------------------------------------------------------------
# Global Data
#-------------------------------------------------------------------------------

MBPS = '|'     # MenuBar path separator

#
# MenuBar Item Types
#
MBTypeCascade     = 'cascade'         # menu item to hold menu items
MBTypeCommand     = 'command'         # command menu item
MBTypeCheckButton = 'checkbutton'     # check button command menu item
MBTypeRadioButton = 'radiobutton'     # radio button command menu item
MBTypeSeparator   = 'separator'       # line separator menu item

#
# Look and Feel
#
ColorGreenDark      = '#009900'
ColorGreen          = 'green'
ColorGreen1         = '#208020'
ColorGreenBlue      = '#00FF80'
ColorBlueGray       = '#8080FF'
ColorRed            = 'red'
ColorRed1           = '#990000'
ColorPink1          = '#990066'
ColorOrange         = 'orange'
ColorBlack          = 'black'
ColorGray1          = '#333333'
ColorGray2          = '#666666'
ColorGray3          = '#999999'
ColorWhite          = 'white'
ColorBlue           = 'blue'
ColorBlue1          = '#000099'
ColorGold1          = '#996600'
ColorRNRYellow      = '#fed700'

# Button text colors
ColorBttnDft        = ColorBlack    # default button text color
ColorBttnGo         = ColorGreen1   # 'go' button
ColorBttnStop       = ColorRed      # 'stop' button
ColorBttnQuit       = ColorRed      # 'quit' button

# Status text colors
ColorFgStatusPrompt = ColorGreen1   # prompt the user
ColorFgStatusOk     = ColorBlack    # ok (normal) status
ColorFgStatusError  = ColorRed1     # error status

# INI text colors
ColorFgIniSection   = ColorPink1    # section name
ColorFgIniOption    = ColorBlack    # option name
ColorFgIniValue     = ColorBlue1    # option value
ColorFgIniComment   = ColorGreen1   # comments
ColorFgIniPunctuation = ColorGray2  # Ini Punctuation

# Shell text colors
ColorFgShellDft     = ColorBlack    # default color
ColorFgShellErr     = ColorRed1     # error message
ColorFgShellPun     = ColorPink1    # shell punctuation
ColorFgShellKey     = ColorGold1    # dictionary key
ColorFgShellNum     = ColorBlue1    # int or float
ColorFgShellStr     = ColorGreen1   # string
ColorFgShellBool    = ColorBlue1    # int or float
ColorFgShellHelp    = ColorGreen1   # help text


# Fonts
FontHelv6           = ('helvetica', 6)
FontHelv10Bold      = ('helvetica', 10, 'bold')
FontHelv12Bold      = ('helvetica', 12, 'bold')
FontHelv24Bold      = ('helvetica', 24, 'bold')
FontCour10Bold      = ('courier', 10, 'bold')
FontCour10          = ('courier', 10, 'normal')


#
# Locations and Files
#
ImageDir            = 'Gui/Images'
ImageRNRLogo        = 'RNRLogo.gif'
ImageInfo           = 'Blocky_Info.gif'
ImageWarning        = 'Blocky_Warning.gif'
ImageError          = 'Blocky_Error.gif'
ImageKhepera        = 'RNRKhepera.gif'
ImageHemisson       = 'RNRHemisson.gif'
ImageKHR1           = 'RNRKHR-1.gif'

#
# Greek Unicodes
#
UniGreek = {
  'Alpha':  u'\u0391', 'Beta':    u'\u0392', 'Gamma':   u'\u0393',
  'Delta':  u'\u0394', 'Epsilon': u'\u0395', 'Zeta':    u'\u0396',
  'Eta':    u'\u0397', 'Theta':   u'\u0398', 'Iota':    u'\u0399',
  'Kappa':  u'\u039A', 'Lambda':  u'\u039B', 'Mu':      u'\u039C',
  'Nu':     u'\u039D', 'Xi':      u'\u039E', 'Omicron': u'\u039F',
  'Pi':     u'\u03A0', 'Rho':     u'\u03A1', 'Sigma':   u'\u03A3',
  'Tau':    u'\u03A4', 'Upsilon': u'\u03A5', 'Phi':     u'\u03A6',
  'Chi':    u'\u03A7', 'Psi':     u'\u03A8', 'Omega':   u'\u03A9',

  'alpha':  u'\u03B1', 'beta':    u'\u03B2', 'gamma':   u'\u03B3',
  'delta':  u'\u03B4', 'epsilon': u'\u03B5', 'zeta':    u'\u03B6',
  'eta':    u'\u03B7', 'theta':   u'\u03B8', 'iota':    u'\u03B9',
  'kappa':  u'\u03BA', 'lambda':  u'\u03BB', 'mu':      u'\u03BC',
  'nu':     u'\u03BD', 'xi':      u'\u03BE', 'omicron': u'\u03BF',
  'pi':     u'\u03C0', 'rho':     u'\u03C1', 'sigma':   u'\u03C3',
  'tau':    u'\u03C4', 'upsilon': u'\u03C5', 'phi':     u'\u03C6',
  'chi':    u'\u03C7', 'psi':     u'\u03C8', 'omega':   u'\u03C9',
}

#
# Subscript Unicodes (usage of these are discouraged by unicode.org)
#
UniSubscript = {
  '0': u'\u2080', '1': u'\u2081', '2': u'\u2082', '3': u'\u2083',
  '4': u'\u2084', '5': u'\u2085', '6': u'\u2086', '7': u'\u2087',
  '8': u'\u2088', '9': u'\u2089', '+': u'\u208A', '-': u'\u208B',
  '=': u'\u208C', '(': u'\u208D', ')': u'\u208E',
}

#
# Superscript Unicodes (usage of these are discouraged by unicode.org)
#
UniSuperscript = {
  '0': u'\u2070', '1': u'\u00b9', '2': u'\u00b2', '3': u'\u00b3',
  '4': u'\u2074', '5': u'\u2075', '6': u'\u2076', '7': u'\u2077',
  '8': u'\u2078', '9': u'\u2079', '+': u'\u207A', '-': u'\u207B',
  '=': u'\u207C', '(': u'\u207D', ')': u'\u207E',
}

#
# Math Unicodes
# see http://www.fileformat.info/info/unicode/category/Sm/list.htm
#
UniMath = {
  '+-':             u'\u00B1', # plus or minus
  'forall':         u'\u2200', # for all
  'pdiff':          u'\u2202', # partial differential
  'thereexists':    u'\u2203', # there exists
  'squareroot':     u'\u221A', # square root
  'infinity':       u'\u221E', # infinity
  'integral':       u'\u222B', # indefinite integral
}

#-------------------------------------------------------------------------------
# Functions
#-------------------------------------------------------------------------------

_MBSepCnt = 0

#--
def MBMakeSepLabel(mbPath=None):
  """ Make MenuBar separation label. Using this function guarantees
      separator label uniqueness in a menubar.

      Parameters:
        mbPath  - 
        mbPath  - a '|' separated path string specifiying where the new
                  sepearator type will be added.   

      Return Value:
        None
  """
  global _MBSepCnt
  if not mbPath:
    mbPath = '#sep%d' % (_MBSepCnt)
  elif mbPath.find('#sep') >= 0:
    return mbPath
  else:
    mbPath += '%s#sep%d' % (MBPS, _MBSepCnt)
  _MBSepCnt += 1
  return mbPath


