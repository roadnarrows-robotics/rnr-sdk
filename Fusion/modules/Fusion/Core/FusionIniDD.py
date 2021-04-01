################################################################################
#
# FusionIniDD.py
#

""" Fusion 'Ini' Definition Dictionary.

Fusion 'Ini' specific definition dictionary.

Format:

inidefdict:
  {sections}
  
sections:
  section
  section sections
  
section:
  'section': [descstring, {options}]

options:
  option
  option options

option:
  'option': [default, descstring]

Author: Robin D. Knight
Email:  robin.knight@roadnarrowsrobotics.com
URL:    http://www.roadnarrowsrobotics.com
Date:   2005.12.16

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

IniDDSectMain  = 'Fusion'
IniDDSectWin   = IniDDSectMain + '/' + 'windows'

#-------------------------------------------------------------------------------
# Functions
#-------------------------------------------------------------------------------

#--
def GetIniDD():
  """ Get Fusion's Ini Definition Dictionary.

      Return Value:
        Ini definition dictionary.
  """
  iniDD = {
    # section
    IniDDSectMain: ['Fusion settings',
    {
      'DebugLevelFusion': [0,           'Debug level of Fusion.'],
      'DebugLevelRobot':  [0,           'Debug level of Robot.'],
      'DebugLevelBrain':  [0,           'Debug level of Brain.'],
      'DebugFileName':    ['<stdout>',  'Debug output filename.'],
      'RobotPluginPath':  [None,        'vRobot directory search path for '
                                        'python plugins.'],
      'BrainPluginPath':  [None,        'vBrain directory search path for '
                                        'python plugins.'],
      'AutoSave':         [True,        'Do [not] autosave ini on exit.'],
      'AutoSavePlugins':  [True,        'Do [not] autosave the last plugins '
                                        'of the session.'],
      'AutoPlugin':       [False,       'Do [not] auto-plugin the <x>Plugins.'],
      'RobotPlugin':      [None,        'vRobot auto-plugin.'],
      'BrainPlugin':      [None,        'vBrain auto-plugin.'],
    }],

    # section
    IniDDSectWin: ['Fusion and child windows settings',
    {
      'Win_Fusion':       [None,        'Fusion root window settings.'],
      'CWin_Fusion':      [None,        'Fusion owned child windows settings']
    }]
  }

  return iniDD

#--
def MergeIniDD(fusionDD, serverDD):
  """ Merge Fusion's current state of its ini Definition Dictionary with 
      the server's DD. Fusion's DD takes precedence.

      Parameters:
        fusionDD  - Fusion's current ini definition dictionary
        serverDD  - server's ini definition dictionary

      Return Value:
        Merged fusion DD
  """
  if not serverDD:
    return fusionDD
  for iniSSection, iniSSectData in serverDD.items():
    if iniSSection not in fusionDD:
      fusionDD[iniSSection] = iniSSectData
    else:
      iniCOptDict = fusionDD[iniSSection][1]
      iniSOptDict = iniSSectData[1]
      for iniSOption, iniSOptData in iniSOptDict.items():
        if iniSOption not in iniCOptDict:
          iniCOptDict[iniSOption] = iniSOptData
  return fusionDD
