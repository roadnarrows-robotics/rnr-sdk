################################################################################
#
# GuiWinIniListing
#

""" Graphical User Interface Ini Listing Window

Graphical User Interface (GUI) Tkinter window to show listing of
the parsed 'ini' configuration data.

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

import  time
import  Tkinter as tk
import  tkFont
import  Fusion.Utils.Tools as utils
import  Fusion.Gui.GuiWinText as GuiWinText
import  Fusion.Gui.GuiDlgSaveAs as GuiDlgSaveAs
import  Fusion.Gui.GuiTypes as gt

#-------------------------------------------------------------------------------
# Global Data
#-------------------------------------------------------------------------------

#-------------------------------------------------------------------------------
# CLASS: GuiWinIniListing
#-------------------------------------------------------------------------------
class GuiWinIniListing(GuiWinText.GuiWinText):
  """ GUI Window Ini Listing Class """

  #--
  def __init__(self, parent, ini, version, iniDD=None, **options):
    """ Initialize the window.

        Parameters:
          parent    - GUI parent of this window
          ini       - IniParser object
          iniDD     - ini definition dictionary
          options   - GuiWin core options
    """
    self.mIni     = ini
    self.mIniDD   = iniDD

    options['create_server'] = False

    GuiWinText.GuiWinText.__init__(self, parent, 
                    title='Ini Configuration Listing',
                    maxHistory=0,   # no maximum
                    fontTuple=gt.FontCour10Bold,
                    lineNums=False,
                    **options)

    self.TagAdd("section", foreground=gt.ColorFgIniSection)
    self.TagAdd("option", foreground=gt.ColorFgIniOption)
    self.TagAdd("value", foreground=gt.ColorFgIniValue)
    self.TagAdd("comment", foreground=gt.ColorFgIniComment)
    self.TagAdd("punctuation", foreground=gt.ColorFgIniPunctuation)

    self.Listing(version)

  #--
  def Listing(self, version):
    """ Display 'ini' listing with comments and defaults. """
    # File Comment Block
    fusionTitle = 'Fusion Configuration File v%s' % version
    ltime = time.localtime()
    date = '%d.%02d.%02d' % (ltime[0], ltime[1], ltime[2])
    self.ShowCommentBlock(['', fusionTitle, date, ''])

    # [DEFAULT] section
    iniDefaults = self.mIni.defaults()
    if iniDefaults:
      self.ShowSection('DEFAULT')
      for option,value in iniDefaults.iteritems():
        self.ShowOption(option, value)
        self.TextAdd('\n', 'punctuation')

    # [Fusion] and plugins sections
    sectionList = self.mIni.sections()
    for section in sectionList:
      self.ShowSection(section)
      items = self.mIni.items(section)
      for option,value in items:
        optlen = self._optlen(option, value)
        # option = value ; comment
        if optlen < 40:
          self.ShowOption(option, value)
          self.ShowCommentParallel(section, option, optlen)
        # ; comment
        # option = value
        else:
          self.ShowOption(option, value)
          self.TextAdd('\n')
          self.ShowComment(section, option, 40)

  #--
  def ShowSection(self, section):
    """ Display 'ini' section comment block(optional) and name.
          ;
          ; comments
          ;
          [section]

        Parameters:
          section   - section name

        Return Value:
          None
    """
    self.TextAdd('\n', 'punctuation')
    if self.mIniDD and self.mIniDD.has_key(section):
      self.ShowCommentBlock(['', self.mIniDD[section][0], ''])
    self.TextAdd('[', 'punctuation')
    self.TextAdd(section, 'section')
    self.TextAdd(']', 'punctuation')
    self.TextAdd('\n', 'punctuation')

  #--
  def ShowOption(self, option, value):
    """ Display 'ini' option name and value.
          option = value

        Parameters:
          option  - option name
          value   - option value (any type)

        Return Value:
          None
    """
    self.TextAdd(option, 'option')
    self.TextAdd(' = ', 'punctuation')
    self.TextAdd(value, 'value')

  #--
  def ShowCommentParallel(self, section, option, optlen):
    """ Display option comment and default value to the right
        side of the option = value.
          ...    ; comment. Default: value

        Parameters:
          section - section name
          option  - option name

        Return Value:
          None
    """
    dft, comment = self._lookupcomment(section, option)
    if not comment:
      self.TextAdd('\n', 'punctuation')
      return
    space = 40 - optlen
    if space > 0:
      self.TextAdd('%*s' % (space, ' '), 'punctuation')
    else:
      self.TextAdd('\n', 'punctuation')
    # Note: as per RFC822, use ';' for in parallel comment offsets
    self.TextAdd('; %s' % comment, 'comment')
    self.TextAdd(' Default: %s\n' % repr(dft), 'comment')

  #--
  def ShowComment(self, section, option, indent):
    """ Display option comment at start of line. 
          ; comment. Default: value

        Parameters:
          section - section name
          option  - option name
          indent  - indentation

        Return Value:
          None
    """
    dft, comment = self._lookupcomment(section, option)
    if not comment:
      return
    if indent > 0:
      self.TextAdd('%*s' % (indent, ' '), 'punctuation')
    self.TextAdd('; %s' % comment, 'comment')
    self.TextAdd(' Default: %s\n' % repr(dft), 'comment')

  #--
  def ShowCommentBlock(self, commentList):
    """ Display comment block.

        Parameters:
          commentList - list of comment lines in block

        Return Value:
          None
    """
    for comment in commentList:
      self.TextAdd('; %s\n' % comment, 'comment')

  #--
  def CbSaveAs(self):
    """ Save As callback. """
    dlg = GuiDlgSaveAs.GuiDlgSaveAs(self, self.Save,
                  title='Save Text As',
                  filetypes=[('Configuration files', '*.ini', 'TEXT'),
                             ('Text files', '*.txt', 'TEXT'),
                             ('All files', '*')],
                  defaultextension='.ini')

  #--
  def _optlen(self, option, value):
    """ Get 'option = value' character length. """
    return len(option) + 3 + len(value)

  #--
  def _lookupcomment(self, section, option):
    """ Get the option default value and comment string from the 
        definition dictionary.
    """
    if not self.mIniDD:
      return None, None
    if not self.mIniDD.has_key(section):
      return None, None
    optDict = self.mIniDD[section][1]
    if not optDict.has_key(option):
      return None, None
    else:
      return optDict[option][0], optDict[option][1]


#-------------------------------------------------------------------------------
# Test Code
#-------------------------------------------------------------------------------

if __name__ == '__main__':
  import  Fusion.Core.Values as Values
  import  Fusion.Utils.IniParser as IniParser
  import  Fusion.Khepera.Robots.KheIniDD as KheIniDD

  #--
  def main():
    """ GuiWinIniListing Test Main """
    inifile = utils.makefilename(Values.FusionPkgDir, 'tests/ini/test1.ini')
    ini = IniParser.IniParser()
    ini.IniOpen(inifile)

    iniDD = KheIniDD.GetIniDD()
    root = tk.Tk()
    win = GuiWinIniListing(root, ini, '0.9', iniDD=iniDD)
    root.mainloop()

  # run test
  main()
