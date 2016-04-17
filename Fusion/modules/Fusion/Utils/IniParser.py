################################################################################
#
# IniParser.py
#

""" Ini Configuration File Utilities

Support interfaces to the ConfigParser package.

Author: Robin D. Knight
Email:  robin.knight@roadnarrowsrobotics.com
URL:    http://www.roadnarrowsrobotics.com
Date:   2005.12.06

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

import os
import sys
import ConfigParser
import re
import Fusion.Utils.Tools as utils

#-------------------------------------------------------------------------------
# Global Data
#-------------------------------------------------------------------------------


#-------------------------------------------------------------------------------
# CLASS: IniNullObj
#-------------------------------------------------------------------------------
class IniNullObj:
  """ Ini Null Object Class. The class instance NullObj is return when no 
      section,option is found. 

      Note: This is distinct from None, '', False, 0 which are all valid
            option values.
  """
  pass

# The Ini Null Object
NullObj = IniNullObj()

#-------------------------------------------------------------------------------
# CLASS: IniParser
#-------------------------------------------------------------------------------
class IniParser(ConfigParser.ConfigParser):
  """ Ini Configuration Parser Case-Sensitive Class. """

  # A copy Ini Null Object for convenience
  NullObj = NullObj

  def __init__(self, cbOnModify=None, defaults=None):
    """ Initialize Ini Parser.

        Parameters:
          cbOnModify  - callback function, called when modified flag
                        transitions
          default     - instrinsic defaults (see ConfigParser)

    """
    ConfigParser.ConfigParser.__init__(self, defaults)
    self.mCbOnModify    = cbOnModify
    self.mModifiedFlag  = False

  # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  # Public Interface 
  # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

  #--
  def optionxform(self, option):
    """ Override ConfigParser.optionxform which converts option names
        to lower case. We want a case-sensitive ini.
    """
    return option

  #--
  def IsModified(self):
    """ Checks if current ini has been modified since object creation
        or last clearing.

        Return Value:
          True if modified, else False.
    """
    return self.mModifiedFlag

  #--
  def ClearModifiedFlag(self):
    """ Clear ini modified flag. If transition from modified, make
        callback.

        Return Value:
          None.
    """
    oldFlag = self.mModifiedFlag
    self.mModifiedFlag = False
    if oldFlag == True and self.mCbOnModify:
      self.mCbOnModify(self.mModifiedFlag)

  #--
  def SetModifiedFlag(self):
    """ Set ini modified flag. If transition from clear, make
        callback.

        Return Value:
          None.
    """
    oldFlag = self.mModifiedFlag
    self.mModifiedFlag = True
    if oldFlag == False and self.mCbOnModify:
      self.mCbOnModify(self.mModifiedFlag)

  #--
  def SetOnModifyCallback(self, cbOnModify):
    """ Set callback function called on modified flag transition.

        Return Value:
          None.
    """
    self.mCbOnModify = cbOnModify

  #--
  def IniOpen(self, iniFileList):
    """ Open and parse ini configuration files from list.

       Parameters:
         iniFileList   - a single filename or list of ini filenames to parse.
                         Any environmental variables ($<var>) or user 
                         specials ('~') will be expanded.

       Return Value:
         List of successfully parsed files.
    """
    # make single filename a list
    if type(iniFileList) != list:
      if iniFileList:
        iniFileList = [iniFileList]
      else:
        return []
    elif not iniFileList:
      return []
  
    # expand file names in list
    expandedFileList = []
    for iniFile in iniFileList:
      iniFile = utils.canonicalpath(iniFile)    # expand $var, ~, strip '//'
      if iniFile:
        expandedFileList += [iniFile]           # append

    # versions older than 2.4 do not return parsed file list, so deal
    if sys.version_info[0] * 10 + sys.version_info[1] < 24:
      self.read(expandedFileList)
      return expandedFileList
    else:
      return self.read(expandedFileList)

  #--
  def IniWrite(self, filename):
    """ Write out ini configuration data to filename.
  
        Parameters:
          filename   - file name
  
        Return Value:
          None.
    """
    fp = open(filename, 'w')
    self.write(fp)
    fp.close()

  #--
  def IniGetReItems(self, section, reOpt):
    """ Get all option items (name, value) in section with names that 
        match the regualar expression.
  
        Parameters:
          section   - section name
          reOpt     - option name regular expression
  
        Return Value:
          List of (name, evaluated_value)'s of all matched option items.
    """
    try:
      itemList = self.items(section)
    except ConfigParser.NoSectionError:
      return []
    matchList = []
    for name,value in itemList:
      if re.match(reOpt, name):
        matchList += [(name, value)]
    return self._EvalItems(matchList)

  #--
  def IniGetSubItems(self, section, optList):
    """ Get all option items (name, value) in section with names that 
        match the option names in the list.
  
        Parameters:
          section   - section name
          optList   - list of option names to retrieve
  
        Return Value:
          List of (name, evaluated_value)'s of all matched option items.
    """
    try:
      itemList = self.items(section)
    except ConfigParser.NoSectionError:
      return []
    matchList = []
    for name,value in itemList:
      if optList.count(name) > 0:
        matchList += [(name, value)]
    return self._EvalItems(matchList)

  #--
  def IniGetItems(self, section):
    """ Get all option items (name, value) in section.
  
        Parameters:
          section   - section name
  
        Return Value:
          List of (name, evaluated_value)'s of all matched option items.
    """
    try:
      itemList = self.items(section)
    except ConfigParser.NoSectionError:
      return []
    return self._EvalItems(itemList)

  #--
  def IniGet(self, section, option):
    """ Get option value.
  
        Parameters:
          section   - section name
          option    - option name
  
        Return Value:
          Returns evaluated option value if section,option exists.
          Else returns None.
    """
    try:  # try to get the section option
      value = self.get(section, option)
      return self._Eval(value)
    except (ConfigParser.NoSectionError, ConfigParser.NoOptionError):
      return self.NullObj

  #--
  def IniGetOrDft(self, section, option, dftValue):
    """ Get option value or fallback to default value.
  
        Parameters:
          section   - section name
          option    - option name
          dftValue  - default value
  
        Return Value:
          Returns evaluated option value if section,option exists.
          Else returns the provided default value.
    """
    try:  # try to get the section option
      value = self.get(section, option)
      try:  # try to convert string into object
        value = eval(value)
      except (SyntaxError, NameError, ValueError):
        pass
      return value
    except (ConfigParser.NoSectionError, ConfigParser.NoOptionError):
      return dftValue

  #--
  def IniSetModifiedItems(self, section, items):
    """ Set all option item (name, value)'s in section whose values
        have been modified. If section does not exist, then it is created.
        Values are automatically converted to string equivalents.
  
        Parameters:
          section   - section name
          items     - list of option items (name, value)
  
        Return Value:
          Number of modified option items updated.
    """
    modcnt = 0
    for name, newValue in items:
      oldValue = self.IniGet(section, name)
      if newValue != oldValue:
        self.IniSet(section, name, newValue)
        modcnt += 1
    return modcnt

  #--
  def IniSetItems(self, section, items):
    """ Set all option item (name, value)'s in section.  If section 
        does not exist, then it is created. Values are automatically 
        converted to string equivalents.
  
        Parameters:
          section   - section name
          items     - list of option items (name, value)
  
        Return Value:
          None.
    """
    for name, value in items:
      self.IniSet(section, name, value)

  #--
  def IniSet(self, section, option, value):
    """ Set the section option value. If section does not exist,
        then it is created. Values are automatically converted to
        string equivalents.
  
        Parameters:
          section - section name
          option  - option name
          value   - option value
  
        Return Value:
          None
    """
    if section != ConfigParser.DEFAULTSECT and not self.has_section(section):
      self.add_section(section)
    if not type(value) == str:
      value = repr(value)
    self.set(section, option, value)
    self.SetModifiedFlag()

  #--
  def IniRemoveReOptions(self, section, reOpt):
    """ Remove all option items (name, value) in section with names that 
        match the regualar expression.
  
        Parameters:
          section   - section name
          reOpt     - option name regular expression
  
        Return Value:
          None.
    """
    try:
      itemList = self.items(section)
    except ConfigParser.NoSectionError:
      return
    for name,value in itemList:
      if re.match(reOpt, name):
        self.remove_option(section, name)
        self.SetModifiedFlag()

  #--
  def IniRemoveSubOptions(self, section, optList):
    """ Remove all option items (name, value) in section with names
        from list.
  
        Parameters:
          section   - section name
          optList   - list of option names
  
        Return Value:
          None.
    """
    if section != ConfigParser.DEFAULTSECT and not self.has_section(section):
      return
    for option in optList:
      self.remove_option(section, option)
      self.SetModifiedFlag()

  #-- 
  def IniRemoveAll(self):
    """ Remove all data from parsed ini.
  
        Return Value:
          None
    """
    # remove all section (and their options) sans DEFAULT
    for section in self.sections():
      self.remove_section(section)
      self.SetModifiedFlag()

    # can't remove DEFAULT, so remove all of its options
    for option,value in self.items(ConfigParser.DEFAULTSECT):
      self.remove_option(ConfigParser.DEFAULTSECT, option)
      self.SetModifiedFlag()

  #--
  def IniSync(self, otherIni):
    # synchronize ini configurations, the client ini takes precedence
    """ Synchronize another ini configuration with this one. This ini
         takes precedence over the other ini. That is, if both ini's
         have identical section,option, this ini value is retained.

         The DEFAULT section is not synchronized.
  
        Parameters:
          otherIni  - IniParser or ConfigParser object

        Return Value:
          Number of merges made.
    """
    numMerges = 0
    if otherIni:
      sectionList = otherIni.sections()
      for section in sectionList:
        items = otherIni.items(section)
        for option,value in items:
          if self.IniGet(section, option) == self.NullObj:
            self.IniSet(section, option, value)
            numMerges += 1
    return numMerges


  # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  # Private Interface 
  # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

  #--
  def _EvalItems(self, items):
    """ Convert options items to (option, evaluated value). """
    evalItems = []
    for option,value in items:
      evalItems += [(option, self._Eval(value))]
    return evalItems

  #--
  def _Eval(self, value):
    """ Convert options value evaluated value. """
    try:
      return eval(value)
    except (SyntaxError, NameError, ValueError):
      return value
