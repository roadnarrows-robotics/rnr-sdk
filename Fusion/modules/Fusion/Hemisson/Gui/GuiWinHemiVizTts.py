################################################################################
#
# GuiWinHemiVizTts.py
#

""" Graphical User Interface Hemisson TTS Visualizer Window

Graphical User Interface (GUI) Tkinter Text-To-Speech visualizer window.
The TTs visualizer displays the text sent to the Hemisson TTS module for 
conversion and speaking.

Author: Robin D. Knight
Email:  robin.knight@roadnarrowsrobotics.com
URL:    http://www.roadnarrowsrobotics.com
Date:   2006.03.11

Copyright (C) 2006.  RoadNarrows LLC.
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

import  Tkinter as tk
import  tkFont

import  Fusion.Core.Values as Values

import  Fusion.Gui.GuiTypes as gt
import  Fusion.Gui.GuiUtils as gut
import  Fusion.Gui.GuiToolTip as GuiToolTip
import  Fusion.Gui.GuiStatusBar as GuiStatusBar
import  Fusion.Gui.GuiWinText as GuiWinText

import  Fusion.Hemisson.Cmd.HemiCmdTts as HemiTts

#-------------------------------------------------------------------------------
# Global Data
#-------------------------------------------------------------------------------

#-------------------------------------------------------------------------------
# CLASS: GuiWinHemiVizTts
#-------------------------------------------------------------------------------
class GuiWinHemiVizTts(GuiWinText.GuiWinText):
  """ GUI Window vHemisson Text-To-Speech Effector Visualizer Class """

  #--
  def __init__(self, parent, **options):
    """ Initialize the Window.

        Parameters:
          parent      - GUI parent of this window
          options     - Visualizer options. Optins are:
            effect_tts=<cb> - effect text-to-speech (say)
            **winoptions    - GuiWinText core options
    """
    # create the window (must be first)
    GuiWinText.GuiWinText.__init__(self, parent,
                                  title='vHemisson Text-To-Speech Visualizer',
                                  maxHistory=100, 
                                  fontTuple=gt.FontHelv24Bold,
                                  height=5,
                                  width=60,
                                  lineNums=False,
                                  **options)

    # first initialization
    self.Init(**options)

  #--
  def Init(self, **options):
    """ First initialization.

        Parameters:
          **options   - visualizer options

        Return Value:
          None
    """
    # defaults
    self.mCbRobotEffectTts  = None

    # set options from input parameters
    for key,val in options.iteritems():
      if key == 'effect_tts':
        self.mCbRobotEffectTts = val

    # override bad options with defaults

    # locals

    # supported text color tags
    self.mColors = {
      'blue':   gt.ColorBlue1,
      'green':  gt.ColorGreen1,
      'red':    gt.ColorRed1,
      'black':  gt.ColorBlack
    }

    # add color tags
    for tag,color in self.mColors.iteritems():
      self.TagAdd(tag, foreground=color)


  # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  # GuiWin Overrides
  # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

  #--
  def body(self):
    """ Initialize the gui window initialization callback. """
    # create text body, menus, etc
    GuiWinText.GuiWinText.body(self)

    # create visualizer specific widgets
    self.GuiBody(self)
 
  #--
  def show(self):
    """ Show the gui window initialization callback. """
    # calculate important window and widget dimensions used for resizing
    self.CalcDim()

  #--
  def resize(self, event):
    """ Resize callback event. """
    # create text body, menus, etc
    GuiWinText.GuiWinText.resize(self, event)
    self.mStatusBar.configure(width=self.mWinGeo[gut.W]-8)


  # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  # Window Update
  # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

  #--
  def WinUpdate(self, request, *args, **kwargs):
    """ Update visualization from the current robot action.

        Parameters:
          request   - the update request 
          *args     - arguments for specific request
          *kwargs   - keyword arguments for specific request

          Specific request and associated parameters:
          'cfg'   - configure TTS window.
                      run_time=<str>  - module's run-time state
                      module=<str>    - module's detected state
                      gain=val        - gain (volume) of TTS
                      pitch=val       - voice pitch to convert speech
                      rate=val        - voice rate (speed) to convert speech
 
          'tts'   - show text that is spoken by the TTS
                      text=val     - text to display
                      colortag=val - displayed text color: supported are:
                                      'red', 'green', 'blue', 'black'

        Return Values:
          None
    """
    #print "%s: WinUpdate: request: %s" % (self.mContextName, request)
    if request == 'cfg':
      items = {}
      for key,val in kwargs.iteritems():
        if key in ['run_time', 'module', 'gain', 'pitch', 'rate']:
          items[key] = val
      self.mStatusBar.Update(**items)
    elif request == 'tts':
      text = kwargs.get('text', '')
      colortag = kwargs.get('colortag', 'black')
      if not self.mColors.has_key(colortag):
        colortag = 'black'
      self.TextAdd(repr(text)+'\n', colortag)
    else:
      print "%s: WinUpdate: unknown request: %s" % (self.mTitle, request)


  # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  # Text-To-Speach Window Specifics
  # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

  #--
  def GuiBody(self, parent):
    """ Create the window body.

        Parameters:
          parent  - parent to this body

        Return Value:
          None
    """
    row = 1
    column = 0
    self.CtlPanelInit(parent, row, column)

  #--
  def CtlPanelInit(self, parent, row, column):
    """ Create the Viz Control Panel.

        Parameters:
          parent  - parent to this frame
          row     - grid row in parent
          column  - grid column in parent

        Return Value:
          None
    """
    # the frame
    cpframe = tk.Frame(parent, relief=tk.FLAT, borderwidth=0)
    cpframe.grid(row=row, column=column, padx=3, ipadx=1, ipady=1, 
               sticky=tk.N+tk.W+tk.E)
    self.mCtlPanelFrame = cpframe

    row = 0
    column = 0
 
    # control panel title
    w = tk.Label(cpframe, text='TTS Control Panel', fg=gt.ColorGreen1)
    w.grid(row=row, column=column)

    row += 1

    # subframe
    subframe = tk.Frame(cpframe, relief=tk.FLAT, borderwidth=0)
    subframe.grid(row=row, column=column, padx=1, pady=1, ipadx=3, ipady=3, 
               sticky=tk.N+tk.W+tk.E)

    subrow = 0
    subcol = 0

    # Say label
    w = tk.Label(subframe, text='Say: ', fg=gt.ColorBlue1)
    w.grid(row=subrow, column=subcol, sticky=tk.W)

    subcol += 1

    # Say text entry box
    w = tk.Text(subframe, height=1, width=HemiTts.TtsMsgLenMax)
    w.grid(row=subrow, column=subcol, sticky=tk.W)
    GuiToolTip.GuiToolTip(w, text="Enter some text to say")
    w.bind('<Return>', self.CbSay)
    self.mTextSay = w

    row += 1

    # status bar width
    self.update_idletasks()
    sbwidth = gut.geometry(cpframe)[gut.W] - 8

    self.mStatusBar = GuiStatusBar.GuiStatusBar(cpframe,
        [ 
          {'tag': 'run_time',
           'prefix': 'run-time:',
           'max_width': 8,
           'val': 'unknown',
           'tooltip': "Module's run-time state.\nSee Robot|Robot Options..."
          },
          {'tag': 'module',
           'max_width': 12,
           'val': 'unknown',
           'tooltip': "Module's detected presence on the Hemisson"
          },
          {'tag': 'gain',
           'max_width': 1,
           'val': HemiTts.TtsSpeakerGainDft,
           'tooltip': "%d = loudest, %d = quietest" % \
                        (HemiTts.TtsSpeakerGainMax, HemiTts.TtsSpeakerGainMin) 
          },
          {'tag':'pitch',
            'max_width': 1,
            'val': HemiTts.TtsVoicePitchDft,
            'tooltip': "%d = highest, %d = lowest" % \
                        (HemiTts.TtsVoicePitchMax, HemiTts.TtsVoicePitchMin) 
          },
          {'tag':'rate',
            'max_width': 1,
            'val': HemiTts.TtsVoiceRateDft,
            'tooltip': "%d = slowest, %d = fastest" % \
                        (HemiTts.TtsVoiceRateMin, HemiTts.TtsVoiceRateMax) 
          },
        ],
        initWidth=sbwidth)
    self.mStatusBar.grid(row=row, column=column, pady=3)


  # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  # Gui Window Callbacks
  # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

  #--
  def CbSay(self, event):
    """ <Return> on TextSay callback. """
    text = self.mTextSay.get(1.0, tk.END)
    self.mTextSay.delete(1.0, tk.END)
    text = text.replace('\n', '')
    text = text.replace('\r', '')
    text = text.replace('\t', ' ')
    text = text.strip()
    if self.mCbRobotEffectTts:
      self.mCbRobotEffectTts(text)


#-------------------------------------------------------------------------------
# Unit Test Code
#-------------------------------------------------------------------------------

if __name__ == '__main__':
  import random
  import Fusion.Utils.WinUT as WinUT
  import Fusion.Utils.IVTimer as IVTimer

  #--
  class WinUTVizTts(WinUT.WinUT):
    """ GuiWinHemiVizTts Unit Test Window """

    #--
    def __init__(self):
      # the unit test
      ut = {
        'Configure Test': self.utCfg,
        'Random Say Test': self.utRandSay
      }

      WinUT.WinUT.__init__(self, title="GuiWinHemiVizTts Unit Test", ut=ut)


    #--
    def PhakeTts(self, spokenText, writtenText=None, writtenColor='black'):
      """ Fake Hemisson TTS"""
      self.wut_showstatus("tts is now speaking: %s (%s)" % \
          (repr(spokenText), writtenColor))
      if not writtenText:
        writtenText = spokenText
      self.mSut.WinQueueRequest('tts', text=writtenText, colortag=writtenColor)

    #--
    # THE UNIT TEST
    #--

    #--
    def utCfg(self):
      """ Configure UT Start """
      cfg = {'run_time': 'disabled', 'module': 'detected', 'gain':2,
             'rate': 1, 'pitch': 7}
      self.wut_showstatus("Configure: %s" % repr(cfg))
      self.mSut.WinQueueRequest('cfg', **cfg)

    #--
    def utRandSay(self):
      """ Randomly say something. """
      color = random.choice(['red', 'green', 'blue', 'black'])
      txt = random.choice(['hello', 'oooga booga',
        'now i lay me down to sleep'])
      self.PhakeTts(txt, writtenColor=color)

  #--
  def main():
    """ GuiWinHemiVizTts Unit Test Main """
    winUT = WinUTVizTts()
    winSut = GuiWinHemiVizTts(winUT.wut_this(), effect_tts=winUT.PhakeTts,
        restState=tk.NORMAL)
    winUT.wut_mark_sut(winSut)
    winUT.wut_this().mainloop()
    winUT.wut_cancel()

  # run unit test
  main()
