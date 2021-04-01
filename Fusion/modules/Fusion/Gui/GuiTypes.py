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
  'Alpha':  '\u0391', 'Beta':    '\u0392', 'Gamma':   '\u0393',
  'Delta':  '\u0394', 'Epsilon': '\u0395', 'Zeta':    '\u0396',
  'Eta':    '\u0397', 'Theta':   '\u0398', 'Iota':    '\u0399',
  'Kappa':  '\u039A', 'Lambda':  '\u039B', 'Mu':      '\u039C',
  'Nu':     '\u039D', 'Xi':      '\u039E', 'Omicron': '\u039F',
  'Pi':     '\u03A0', 'Rho':     '\u03A1', 'Sigma':   '\u03A3',
  'Tau':    '\u03A4', 'Upsilon': '\u03A5', 'Phi':     '\u03A6',
  'Chi':    '\u03A7', 'Psi':     '\u03A8', 'Omega':   '\u03A9',

  'alpha':  '\u03B1', 'beta':    '\u03B2', 'gamma':   '\u03B3',
  'delta':  '\u03B4', 'epsilon': '\u03B5', 'zeta':    '\u03B6',
  'eta':    '\u03B7', 'theta':   '\u03B8', 'iota':    '\u03B9',
  'kappa':  '\u03BA', 'lambda':  '\u03BB', 'mu':      '\u03BC',
  'nu':     '\u03BD', 'xi':      '\u03BE', 'omicron': '\u03BF',
  'pi':     '\u03C0', 'rho':     '\u03C1', 'sigma':   '\u03C3',
  'tau':    '\u03C4', 'upsilon': '\u03C5', 'phi':     '\u03C6',
  'chi':    '\u03C7', 'psi':     '\u03C8', 'omega':   '\u03C9',
}

#
# Subscript Unicodes (usage of these are discouraged by unicode.org)
#
UniSubscript = {
  '0': '\u2080', '1': '\u2081', '2': '\u2082', '3': '\u2083',
  '4': '\u2084', '5': '\u2085', '6': '\u2086', '7': '\u2087',
  '8': '\u2088', '9': '\u2089', '+': '\u208A', '-': '\u208B',
  '=': '\u208C', '(': '\u208D', ')': '\u208E',
}

#
# Superscript Unicodes (usage of these are discouraged by unicode.org)
#
UniSuperscript = {
  '0': '\u2070', '1': '\u00b9', '2': '\u00b2', '3': '\u00b3',
  '4': '\u2074', '5': '\u2075', '6': '\u2076', '7': '\u2077',
  '8': '\u2078', '9': '\u2079', '+': '\u207A', '-': '\u207B',
  '=': '\u207C', '(': '\u207D', ')': '\u207E',
}

#
# Math Unicodes
# see http://www.fileformat.info/info/unicode/category/Sm/list.htm
#
UniMath = {
  '+-':             '\u00B1', # plus or minus
  'forall':         '\u2200', # for all
  'pdiff':          '\u2202', # partial differential
  'thereexists':    '\u2203', # there exists
  'squareroot':     '\u221A', # square root
  'infinity':       '\u221E', # infinity
  'integral':       '\u222B', # indefinite integral
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


