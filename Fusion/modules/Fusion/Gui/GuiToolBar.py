################################################################################
#
# GuiToolBar.py
#

""" Graphical User Interface ToolBar Module

Graphical User Interface (GUI) Tkinter toolbar module.

Author: Robin D. Knight
Email:  robin.knight@roadnarrowsrobotics.com
URL:    http://www.roadnarrowsrobotics.com
Date:   2005.12.08

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

import  Tkinter as tk
import  Fusion.Gui.GuiTypes as gt
import  Fusion.Gui.GuiToolTip as GuiToolTip

#-------------------------------------------------------------------------------
# Global Data
#-------------------------------------------------------------------------------

#-------------------------------------------------------------------------------
# CLASS: GuiToolBar
#-------------------------------------------------------------------------------
class GuiToolBar:
  """ GUI ToolBar Class
      RDK TODO: change to a state machine model.
  """

  #--
  def __init__(self, parent, row=0, column=0):
    """ Initialize toolbar instance.

        Parameters:
          parent  - parent Tkinter widget
          row     - grid row in parent
          column  - grid column in parent
    """
    tbframe = tk.Frame(parent, relief=tk.RAISED, borderwidth=1)
    tbframe.grid(row=row, column=column, padx=1, ipadx=3, ipady=3, sticky=tk.W)

    self.mFrame  = tbframe
    self.mColumn  = 0
    self.mToolBar = {}

  #--
  def __del__(self):
    """ Delete toolbar. """
    self.mFrame.destroy()

  #--
  def __iter__(self):
    """ Iterator. """
    return self.mToolBar.iteritems

  #--
  def AddButton(self, label, command, tooltip='', owner='*', disabledStates={},
                      imagefile=None,
                      altStates={}, alttooltip='', altimagefile=None,
                      **kw):
    """ Add a new button to the toolbar. The attachment point is at 
        the end of the last item in the toolbar. The toolbar item cannot 
        already exist.

        Parameters:
          label     - button unique identifier. Doubles as button text
                      if button no bitmap or image.
          command   - button callback.
          tooltip   - brief description of button
          owner     - button owner. '*' = any
          disabledStates
                    - dictionary of state spaces. Each space has a list
                      of disabled states whose elements can be any 
                      comparable object.
          imagefile - default button image file
          altStates
                    - dictionary of state spaces. Each space has a list
                      of alternate states whose elements can be any 
                      comparable object.
          alttooltip - alternate description of button
          altimagefile - alternate button image file
          kw        - appropriate keyword=value arguments for the given 
                      TkInter.Button item type.
        
        Exceptions:
          KeyError    if button already exists
          TclError    for any associated Tkinter error.

        Return Value:
          None.
    """
    if self.mToolBar.has_key(label):
      raise KeyError("Toolbar item already exists: %s" % (repr(label)))

    # default button image, if any
    if imagefile:
      img = tk.PhotoImage(file=imagefile)
      bttn = tk.Button(self.mFrame, text=label, command=command, image=img,
                                     **kw)
      bttn.imageDft = img # must keep a reference, or image will be deleted on
                          # this function's return
      curImage = 'default'
    else:
      bttn = tk.Button(self.mFrame, text=label, command=command, **kw)
      curImage = 'none'
 
    # alternate button image, if any
    if altimagefile:
      img = tk.PhotoImage(file=altimagefile)
      bttn.imageAlt = img # must keep a reference, or image will be deleted on
                          # this function's return

    bttn.grid(row=0, column=self.mColumn, sticky=tk.W)
    self.mColumn += 1
    if tooltip or alttooltip:
      tt = GuiToolTip.GuiToolTip(bttn, text=tooltip)
    else:
      tt = None
    self.mToolBar[label] = {'button':bttn,
                            'tooltip': tt,
                            'tip':tooltip,
                            'owner':owner,
                            'disabledStates':disabledStates,
                            'activeDisabled':[],
                            'altStates':altStates,
                            'alttip':alttooltip,
                            'curImage':curImage}

  #--
  def AddSpace(self, space=1):
    """ Add space before next button.

        Parameters:
          space - amount of whitespace to insert.

        Return Value:
          None
    """
    l = tk.Label(self.mFrame, text='%*s' % (space, ' '))
    l.grid(row=0, column=self.mColumn, sticky=tk.W)
    self.mColumn += 1

  #--
  def SetButtonStates(self, stateSpace, state):
    """ Set toolbar buttons to NORMAL/DISABLED states.

        Parameters:
          stateSpace  - state value space
          state       - any comparable ('==') object value within the
                        state space

        Return Value:
          None.
    """
    # iterate over all buttons in toolbar
    for label,item in self.mToolBar.iteritems():

      # check if the state space applies to this button
      if item['disabledStates'].has_key(stateSpace):

        # the state matches a value in the state space
        if item['disabledStates'][stateSpace].count(state) > 0:
          # disable the button
          if item['button']['state'] != tk.DISABLED:
            item['button']['state'] = tk.DISABLED
          # add the state space to the disabled list
          if item['activeDisabled'].count(stateSpace) == 0:
            item['activeDisabled'].append(stateSpace)

        # the state does not match a value in the state space
        else:
          # remove from the disabled list
          if item['activeDisabled'].count(stateSpace) > 0:
            idx = item['activeDisabled'].index(stateSpace)
            del item['activeDisabled'][idx]
          # if no more disabled spaces in the disabled list, enable the button
          if len(item['activeDisabled']) == 0 and \
              item['button']['state'] != tk.NORMAL:
            item['button']['state'] = tk.NORMAL

      # toggle alternate/default image if applicable
      if item['altStates'].has_key(stateSpace):
        # alternate iamge
        if item['altStates'][stateSpace].count(state) > 0:
          if item['curImage'] != 'alternate':
            bttnstate = item['button']['state']
            item['button']['state'] = tk.NORMAL
            item['button'].config(image=item['button'].imageAlt)
            item['button']['state'] = bttnstate
            if item['tooltip']:
              item['tooltip'].newtip(item['alttip'])
            item['curImage'] = 'alternate'
        # default iamge
        else:
          if item['curImage'] != 'default':
            bttnstate = item['button']['state']
            item['button']['state'] = tk.NORMAL
            item['button'].config(image=item['button'].imageDft)
            item['button']['state'] = bttnstate
            if item['tooltip']:
              item['tooltip'].newtip(item['tip'])
            item['curImage'] = 'default'


#-------------------------------------------------------------------------------
# Test Code
#-------------------------------------------------------------------------------

if __name__ == '__main__':
  image0 = '/prj/src/fusion/Fusion-1.0/Fusion/Gui/Images/SerConn.gif'
  image1 = '/prj/src/fusion/Fusion-1.0/Fusion/Gui/Images/SerDisc.gif'

  tbar = None

  def cbDo():
    print('Do')

  def cbIt():
    print('It')
    tbar.SetButtonStates('funny', 'pie-in-face')

  def cbNow():
    print('Now')
    tbar.SetButtonStates('funny', 'tickle')

  #--
  def main():
    """ GuiToolBar Test Main """
    global tbar
    root = tk.Tk()
    tbar = GuiToolBar(root)
    tbar.AddButton('Do', cbDo)
    tbar.AddButton('It', cbIt)
    tbar.AddButton('Now', cbNow, 
        tooltip='connect', imagefile=image0, 
        altStates={'funny': ['tickle']}, altimagefile=image1,
        alttooltip='disconnect')
    root.mainloop()

  # run test
  main()
