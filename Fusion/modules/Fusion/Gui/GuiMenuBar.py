################################################################################
#
# GuiMenuBar.py
#

""" Graphical User Interface MenuBar Module

Graphical User Interface (GUI) Tkinter menubar module.

Author: Robin D. Knight
Email:  robin.knight@roadnarrowsrobotics.com
URL:    http://www.roadnarrowsrobotics.com
Date:   2005.11.16

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

import  tkinter as tk
import  Fusion.Gui.GuiTypes as gt

#-------------------------------------------------------------------------------
# Global Data
#-------------------------------------------------------------------------------

#-------------------------------------------------------------------------------
# CLASS: GuiMenuBarIter
#-------------------------------------------------------------------------------
class GuiMenuBarIter:
  """ GUI MenuBarItem Iterator Class.  """
  #--
  def __init__(self, mbItemParent, rootPath, what='path', how='topdown'):
    """ Initialized iterator instance. """
    self.mStart = mbItemParent
    self.mWhat  = what 
    self.mHow   = how 
    self.mStack = []
    if self.mWhat == 'path':
      if self.mHow == 'topdown':
        self._push(self.mStart, rootPath)
        self.mNextFunc = self.nextPathTopDown
    else:
      raise ValueError("Unsupported iterator: %s" % (repr(what)))

  #--
  def __iter__(self):
    """ Iterator start. (Not called usually)"""
    return self

  #--
  def __next__(self):
    """ Iterator next. """
    return self.mNextFunc(self._pop())

  #--
  def nextPathTopDown(self, args):
    """ Iterator next for menubar path, top-down order. """
    mbItemParent, mbPath = args
    i = len(mbItemParent['subtrees']) - 1
    while i >= 0:
      mbItem = mbItemParent['subtrees'][i]
      if mbPath != gt.MBPS:
        newPath = mbPath + gt.MBPS + mbItem['label']
      else:
        newPath = mbItem['label']
      self._push(mbItem, newPath)
      i -= 1
    if mbPath == gt.MBPS: # ignore root
      return self.nextPathTopDown(self._pop())
    else:          # this node is good
      return mbPath

  #--
  def _push(self, *args):
    """ Push menubar item plus other data on the stack. """
    self.mStack += [args]

  #--
  def _pop(self):
    """ Pop menubar item plus other data off of the stack. """
    i = len(self.mStack) - 1
    if i >= 0:
      args = self.mStack[i]
      del self.mStack[i]
      return args
    else:
      raise StopIteration


#-------------------------------------------------------------------------------
# CLASS: GuiMenuBarItem
#-------------------------------------------------------------------------------
class GuiMenuBarItem:
  """ GUI MenuBarItem Helper Class.
      RDK TODO: change to a state machine model.
  """

  #--
  def __init__(self, mbItemParent, index):
    """ Initialized helper instance. """
    self.mMenuBarItem  = mbItemParent  # MenuBar item
    self.mIndex        = index         # Tkinter.Menu INDEX

  #--
  def __getitem__(self, option):
    """ Get Tkinter.Menu item configuration option.
   
        Parameters:
          option  - see Tkinter.Menu.entrycget()

        Return Value:
          Option value.
    """
    if not self.mMenuBarItem or self.mIndex <= 0:
      raise TypeError("GuiMenuBarItem is unbound to any menu item")

    mbItem = self.mMenuBarItem['subtrees'][self.mIndex-1]
    type   = mbItem['type']

    # pre-watch options
    if option == 'label':
      if  type == 'tearoff' or \
          type == gt.MBTypeCascade or \
          type == gt.MBTypeSeparator:
        return mbItem['label']
    elif option == 'owner':
        return mbItem['owner']
    elif option == 'type':
        return mbItem['type']
    elif option == 'disabledStates':
        return mbItem['disabledStates']

    return self.mMenuBarItem['menu'].entrycget(self.mIndex, option)

  #--
  def __setitem__(self, option, val):
    """ Set Tkinter.Menu item configuration option.
   
        Parameters:
          option  - see Tkinter.Menu.entryconfigure()
          val     - new value of option

        Return Value:
          None.
    """
    if not self.mMenuBarItem or self.mIndex <= 0:
      raise TypeError("GuiMenuBarItem is unbound to any menu item")

    mbItem = self.mMenuBarItem['subtrees'][self.mIndex-1]
    type   = mbItem['type']

    # pre-watch options
    if option == 'menu':
      raise KeyError("Cannot set 'menu' here, fracks up GuiMenuBar instance")
    elif option == 'label':
      if  type == 'tearoff' or \
          type == gt.MBTypeCascade or \
          type == gt.MBTypeSeparator:
        mbItem['label'] = val
        return
    elif option == 'owner':
        mbItem['owner'] = val
        return
    elif option == 'disabledStates':
        mbItem['disabledStates'] = val
        return

    # weak, but it works
    menu = self.mMenuBarItem['menu']
    expr = "menu.entryconfigure(self.mIndex, %s=%s)" % (option, repr(val))
    eval(expr)

    # post-watch options
    if option == 'label':
      mbItem['label'] = val

  #--
  def _BindItem(self, mbItem, index):
    """ Bind to Tkinter.Menu item.
   
        Parameters:
          mbItem  - menubar item
          index   - Tkinter.Menu INDEX
    """
    self.mMenuBarItem  = mbItem
    self.mIndex        = index


#-------------------------------------------------------------------------------
# CLASS: GuiMenuBar
#-------------------------------------------------------------------------------
class GuiMenuBar:
  """ GUI MenuBar Class. """
  #--
  def __init__(self, parent):
    """ Initialize menubar instance.

        Parameters:
          parent  - parent Tkinter widget
    """
    self.mMenuBar = tk.Menu(parent)
    parent.config(menu=self.mMenuBar)
    self.mMenuBarTree = {
      'label':          gt.MBPS,
      'menu':           self.mMenuBar,
      'type':           self.mMenuBar.type(0),
      'owner':          '*',
      'disabledStates': {},
      'activeDisabled': [],
      'subtrees':       []
    }

  #--
  def __getitem__(self, mbPath):
    """ Get Tkinter.Menu item configuration option.
   
        Synopsis:
          GuiMenuBar[mbPath][option] 
                      <==> Tkinter.Menu.entrycget(index, option)
          
        Parameters:
          mbPath    - a '|' separated path string specifying the menu item.

        Return Value:
          Option value.
    """
    mbItemParent, index = self._FindItem(mbPath, self.mMenuBarTree)
    return GuiMenuBarItem(mbItemParent, index)

  #--
  def __setitem__(self, mbPath):
    """ Set Tkinter.Menu item configuration option.
   
        Synopsis:
          GuiMenuBar[mbPath][option]=val 
                      <==> Tkinter.Menu.entryconfigure(index, option=val)
          
        Parameters:
          mbPath    - a '|' separated path string specifying the menu item.

        Return Value:
          None.
    """
    mbItemParent, index = self._FindItem(mbPath, self.mMenuBarTree)
    return GuiMenuBarItem(mbItemParent, index)

  #--
  def __delitem__(self, mbPath):
    """ Delete menu bar item.

        Synopsis:
          del GuiMenuBar[mbPath] <==> Tkinter.Menu.delete(index)
          
        Parameters:
          mbPath    - a '|' separated path string specifying the menu item.

        Return Value:
          None.
    """
    self.DelMenuItem(mbPath, owner='root')

  #--
  def __iter__(self):
    """ Iterator initializer for menubar path, top-down order.

        Synopsis:
          for p in GuiMenuBar: ... 
          
        Return Value:
          Iterator.
    """
    return GuiMenuBarIter(self.mMenuBarTree, '|', what='path', how='topdown')


  # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  # GuiMenuBar Public Interface 
  # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

  #--
  def iterpaths(self, mbPath=None):
    """ Iterator initializer for menubar path, top-down order.

        Synopsis:
          for p in GuiMenuBar(mbPath): ... 
          
        Parameters:
          mbPath    - menubar path starting point.

        Return Value:
          Iterator.
    """
    if not mbPath:
      mbPath = gt.MBPS
      mbItemStart = self.mMenuBarTree
    else:
      mbItemStart, index = self._FindItem(mbPath, self.mMenuBarTree)
      mbItemStart = mbItemStart['subtrees'][index-1]
    return GuiMenuBarIter(mbItemStart, mbPath, what='path', how='topdown')

  #--
  def AddMenuItem(self, mbPath, itemType, owner='*', disabledStates={}, **kw):
    """ Add a new mbPath menubar item to the menubar. The attachment
        point is at the end of the existing component of mbPath.
        The menubar item cannot already exist.

        Parameters:
          mbPath    - a '|' separated path string. For MBTypeSeparator type,
                      the end 'label' will be automatically generated.
          itemType  - menu item type: One of:                
                        MBTypeCascade, MBTypeCommand, MBTypeSeparator, 
                        MBTypeCheckButton, MBTypeRadioButton
          owner     - menu item's owner. '*' = any
          disabledStates
                    - dictionary of state spaces. Each space has a list
                      of disabled states whose elements can be any 
                      comparable object.
          kw        - appropriate keyword=value arguments for the given 
                      TkInter.Menu item type.
        
        Exceptions:
          SyntaxError if mbPath syntax is invalid.
          KeyError    if mbPath already exist or if it the new mbPath
                      components cannot be attached.
          TclError    for any associated Tkinter error.

        Return Value:
          Returns massaged menu path. 
    """
    mbTree = self.mMenuBarTree

    # add last path compenent for separators that don't really have labels
    if itemType == gt.MBTypeSeparator:
      mbPath = gt.MBMakeSepLabel(mbPath)

    # Find add point
    tailPath, mbItemParent = self._FindAddPoint(mbPath, mbTree)

    # new INDEX
    index = len(mbItemParent['subtrees']) + 1

    # Add menu item(s)
    self._InsMenuItem(mbItemParent, index, tailPath, itemType, owner, 
                      disabledStates, **kw)

    return mbPath

  #--
  def InsMenuItem(self, mbInsPath, mbPath, itemType, owner='*', 
                        disabledStates={}, **kw):
    """ Insert a new mbPath menubar item to the menubar. The attachment
        point is before the last component of mbInsPath.
        The mbInsPath menubar item must already exist.

        ValueError will be raised if:
          Menu path syntax is invalid.
          The insertion point mbInsPath menubar item does not exist.

        Parameters:
          mbInsPath - a '|' separated path string to an existing menubar
                      item.
          mbPath    - a '|' separated path string specifiying the new
                      menubar item(s). For MBTypeSeparator type,
                      the end 'label' will be automatically generated.
          itemType  - menu item type: One of:                
                        MBTypeCascade, MBTypeCommand, MBTypeSeparator, 
                        MBTypeCheckButton, MBTypeRadioButton
          owner     - menu item's owner. '*' = any
          disabledStates
                    - dictionary of state spaces. Each space has a list
                      of disabled states whose elements can be any 
                      comparable object.
          kw        - appropriate keyword=value arguments for the given 
                      TkInter.Menu item type.
        
        Exceptions:
          SyntaxError if menubar path syntax is invalid.
          KeyError    if mbInsPath does not exist.
          TclError    for any associated Tkinter error.

        Return Value:
          Returns massaged menu path. 
    """
    mbTree = self.mMenuBarTree

    # add last path compenent for separators that don't really have labels
    if itemType == gt.MBTypeSeparator:
      mbPath = gt.MBMakeSepLabel(mbPath)

    # Find insertion point
    mbItemParent, index = self._FindItem(mbInsPath, self.mMenuBarTree)

    # Insert menu item(s)
    self._InsMenuItem(mbItemParent, index, mbPath, itemType, owner,
                      disabledStates, **kw)

    return self._ReplaceLastPathComp(mbInsPath, mbPath)

  #--
  def AppMenuItem(self, mbAppPath, mbPath, itemType, owner='*',
                        disabledStates={}, **kw):
    """ Append a new mbPath menubar item to the menubar. The attachment
        point is after the last component of mbAppPath.
        The mbAppPath menubar item must already exist.

        Parameters:
          mbAppPath - a '|' separated path string to an existing menubar
                      item.
          mbPath    - a '|' separated path string specifiying the new
                      menubar item(s). For MBTypeSeparator type,
                      the end 'label' will be automatically generated.
          itemType  - menu item type: One of:                
                        MBTypeCascade, MBTypeCommand, MBTypeSeparator, 
                        MBTypeCheckButton, MBTypeRadioButton
          owner     - menu item's owner. '*' = any
          disabledStates
                    - dictionary of state spaces. Each space has a list
                      of disabled states whose elements can be any 
                      comparable object.
          kw        - appropriate keyword=value arguments for the given 
                      TkInter.Menu item type.
        
        Exceptions:
          SyntaxError if menubar path syntax is invalid.
          KeyError    if mbAppPath does not exist.
          TclError    for any associated Tkinter error.

        Return Value:
          Returns massaged menu path. 
    """
    mbTree = self.mMenuBarTree

    # add last path compenent for separators that don't really have labels
    if itemType == gt.MBTypeSeparator:
      mbPath = gt.MBMakeSepLabel(mbPath)

    # Find insertion point
    mbItemParent, index = self._FindItem(mbAppPath, self.mMenuBarTree)

    # Insert menu item(s)
    self._InsMenuItem(mbItemParent, index+1, mbPath, itemType, owner,
                      disabledStates, **kw)

    return self._ReplaceLastPathComp(mbAppPath, mbPath)

  #--
  def DelMenuItem(self, mbPath, owner='*'):
    """ Delete the mbPath menubar item from the menubar. All mbPath's 
        sub-menubar items will also be deleted. The to-be-delete menubar
        item (and subtrees) must have owner permission.  The menubar item 
        must already exist.

        Parameters:
          mbPath    - a '|' separated path string.
          owner     - owner of caller.
        
        Exceptions:
          SyntaxError if mbPath syntax is invalid.
          KeyError    if mbAppPath does not exist.
          ValueError  if owner permission is denied.
          TclError    for any associated Tkinter error.

        Return Value:
          None.
    """
    # find menubar item
    mbItemParent, index = self._FindItem(mbPath, self.mMenuBarTree)

    # check owner permissions
    if not self._HasPermission(mbItemParent, owner) or \
        not self._HasPermissionTree(mbItemParent['subtrees'][index-1], owner): 
      raise ValueError("Permission denied: owner %s does not fully own %s" \
          % (repr(owner), repr(mbPath)))

    # delete menubar subtree from parent
    self._DelTree(mbItemParent, index)

  #--
  def DelMenuItemsByOwner(self, owner, mbPath=None):
    """ Delete all menubar items from the [sub]menubar tree that the
        owner has permission to delete. Note that the root of the menubar
        is never deleted.

        Parameters:
          mbPath    - a '|' separated path string.
                        (default: root of menubar tree)
          owner     - owner menubar items to be deleted. If owner is:
                        'root' = will delete full mbPath menubar tree
                        <name> = will delete only mbPath menubar items
                                 that match <name>.
        
        Exceptions:
          SyntaxError if mbPath syntax is invalid.
          KeyError    if mbAppPath does not exist.
          ValueError  if owner permission is denied.
          TclError    for any associated Tkinter error.

        Return Value:
          None.
    """
    if mbPath is None:
      index = len(self.mMenuBarTree['subtrees'])
      while index > 0:
        self._DelOwnerTree(owner, self.mMenuBarTree, index)
        index -= 1
    else:
      mbItemParent, index = self._FindItem(mbPath, self.mMenuBarTree)
      self._DelOwnerTree(owner, mbItemParent, index)

  #--
  def GetMenuItem(self, mbPath):
    """ Get the menu and item index of the menubar item identified by 
        the path.

        ValueError will be raised if:
          Menu path syntax is invalid.

        Parameters:
          mbPath  - a '|' separated path string.
        
        Return Value:
          On search success, return tuple (Tkinter.Menu, INDEX)
          On search failure, return (None, -1)
    """
    try:
      mbItemParent, index = self._FindItem(mbPath, self.mMenuBarTree)
      return (mbItemParent['menu'], index)
    except KeyError:
      return (None, -1)

  #--
  def SetMenuItemStates(self, stateSpace, state, mbPath=None):
    """ Set menubar item NORMAL/DISABLED states of the mbPath tree. 

        Parameters:
          stateSpace  - state value space
          state       - any comparable ('==') object value within the
                        state space
          mbPath      - a '|' separated path string.

        Return Value:
          None.
    """
    #print('DBG: stateSpace=%s, state=%s, mbPath=%s' % \
    #    (repr(stateSpace), repr(state), repr(mbPath)))
    for mbPath in self.iterpaths(mbPath):
      mbItemParent, index = self._FindItem(mbPath, self.mMenuBarTree)
      mbItem = mbItemParent['subtrees'][index-1]

      # ignore separators
      if mbItem['type'] == gt.MBTypeSeparator:
        continue

      # check if the state space applies to this menubar item
      if stateSpace in mbItem['disabledStates']:

        # the state matches a value in the state space
        if mbItem['disabledStates'][stateSpace].count(state) > 0:
          # disable the menubar item
          if mbItemParent['menu'].entrycget(index, 'state') != tk.DISABLED:
            mbItemParent['menu'].entryconfigure(index, state=tk.DISABLED)
          # add the state space to the disabled list
          if mbItem['activeDisabled'].count(stateSpace) == 0:
            mbItem['activeDisabled'].append(stateSpace)

        # the state does not match a value in the state space
        else:
          # remove from the disabled list
          if mbItem['activeDisabled'].count(stateSpace) > 0:
            idx = mbItem['activeDisabled'].index(stateSpace)
            del mbItem['activeDisabled'][idx]
          # if no more disabled spaces in the disabled list, enable the item
          if len(mbItem['activeDisabled']) == 0 and \
              mbItemParent['menu'].entrycget(index, 'state') == tk.DISABLED:
            mbItemParent['menu'].entryconfigure(index, state=tk.NORMAL)
      
  #--
  def Split(self, mbPath):
    """ Split menu path into its first label component and trailing subpath.

        Exceptions:
          SyntaxError if mbPath syntax is invalid.

        Return Value:
          label, tailPath
    """
    p = mbPath
    if not p:
      raise ValueError('No MenuBar Path specified')
    while len(p) > 0:
      i = mbPath.find(gt.MBPS)
      if i < 0:           # 'label'
        return p, ''
      elif i == 0:        # '|...'
        p = p[1:]
      else:               # 'label|...'
        return p[:i], p[i+1:]
    raise SyntaxError('Bad MenuBar specified: %s' % (repr(mbPath)))


  # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  # GuiMenuBar Private Interface 
  # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

  #--
  def _InsMenuItem(self, mbItemParent, index, mbPath, itemType, owner, 
                         disabledStates, **kw):
    """ Do insert the new menu item(s) before the given Tkinter.Menu
        INDEX. 

        Parameters:
          mbItemParent    - parent item (add point in menubar tree)
          index           - insertion point in parent
          mbPath          - a '|' separated path string.
          itemType        - menu item type
          owner           - menubar item's owner. '*' = any
          disabledStates  - dictionary of state spaces. Each space has a 
                            list of disabled states whose elements can be 
                            any comparable object.
          kw              - appropriate keyword=value arguments for the
                            given TkInter.Menu item type.
        
        Return Value:
          None. 
    """
    label, tailPath = self.Split(mbPath)

    # create new part of menu tree
    if tailPath: # more path components, so this item has to be a cascade
      mbItemChild = self._InsItem(mbItemParent, index, label,
                                  gt.MBTypeCascade, owner,
                                  disabledStates, **kw)
      self._InsMenuItem(mbItemChild, 1, tailPath, itemType, owner,
                        disabledStates, **kw)
    else: # last menu component
      self._InsItem(mbItemParent, index, label, itemType, owner,
                    disabledStates, **kw)
    
  #--
  def _InsItem(self, mbItemParent, index, label, itemType, owner,
                     disabledStates, **kw):
    if itemType == gt.MBTypeCascade:
      menu = tk.Menu(mbItemParent['menu'])
      mbItemParent['menu'].insert_cascade(index, label=label, menu=menu, **kw)
    elif itemType == gt.MBTypeSeparator:
      mbItemParent['menu'].insert_separator(index, **kw)
      menu = None
    else:
      mbItemParent['menu'].insert(index, itemType, label=label, **kw)
      menu = None
    mbItemParent['subtrees'].insert(index-1,
      {
        'label':          label,
        'menu':           menu,
        'type':           itemType,
        'owner':          owner,
        'disabledStates': disabledStates,
        'activeDisabled': [],
        'subtrees':       []
      })
    return mbItemParent['subtrees'][index-1]

  #--
  def _DelTree(self, mbItemParent, index):
    """ Do delete menubar item (and all its subtrees) from the parent
        menubar item.

        Parameters:
          mbItemParent  - parent menubar item of the to-be-deleted subtree
          index         - Tkinter.Menu INDEX of the to-be-deleted menu item 
        
        Return Value:
          None. 
    """
    if len(mbItemParent['subtrees']) == 0:
      return
    else:
      mbDelItem = mbItemParent['subtrees'][index-1]

    # delete all menubar subtrees of the to-be-deleted item
    # Note: index shifts down after each delete (i.e. remains fixed)
    for mbItem in mbDelItem['subtrees']:
      self._DelTree(mbItem, 1)

    # delete menubar item
    self._DelItem(mbItemParent, index)

  #--
  def _DelOwnerTree(self, owner, mbItemParent, index):
    """ Do delete menubar item (and all its subtrees) from the parent
        menubar item that have the given owner permissions.

        Parameters:
          owner         - owner menubar items to be deleted.
          mbItemParent  - parent menubar item of the to-be-deleted subtree
          index         - Tkinter.Menu INDEX of the to-be-deleted menu item 
        
        Return Value:
          None. 
    """
    # the to-be-deleted item
    mbDelItem = mbItemParent['subtrees'][index-1]

    # delete the to-be-deleted item's subtrees from right to left
    indexChild = len(mbDelItem['subtrees'])
    while indexChild > 0:
      self._DelOwnerTree(owner, mbDelItem, indexChild)
      indexChild -= 1

    # not all of the to-be-deleted item's subtrees could be deleted
    if len(mbDelItem['subtrees']) > 0:
      return

    # if owner or root, then delete this item
    elif owner == 'root' or mbDelItem['owner'] == owner:
      self._DelItem(mbItemParent, index)
      
  #--
  def _DelItem(self, mbItemParent, index):
    """ Do delete menubar item.

        Parameters:
          mbItemParent  - parent menubar item of the to-be-deleted subtree
          index         - Tkinter.Menu INDEX of the to-be-deleted menu item 
        
        Return Value:
          None. 
    """
    # the to-be-deleted item
    # delete from gui
    mbItemParent['menu'].delete(index)

    # delete menubar item
    mbDelItem = mbItemParent['subtrees'][index-1]
    mbDelItem['label']     = ''
    mbDelItem['type']      = ''
    mbDelItem['owner']     = ''
    mbDelItem['subtrees']  = []
    if mbDelItem['menu']:
      del mbDelItem['menu']

    # delete from parent menubar
    del mbItemParent['subtrees'][index-1]

  #--
  def _HasPermission(self, mbItem, owner):
    """ Check owner permission of the menubar item.

        Parameters:
          mbItem  - menubar item to check
          owner   - owner of caller
        
        Return Value:
          True if has permission, False otherwise. 
    """
    if owner == 'root': 
      return True
    elif mbItem['owner'] == '*': 
      return True
    elif mbItem['owner'] == owner:
      return True
    else:
      return False

  #--
  def _HasPermissionTree(self, mbTree, owner):
    """ Check owner permission of the menubar tree.

        Parameters:
          mbTree  - menubar tree to check
          owner   - owner of caller
        
        Return Value:
          True if has permission, False otherwise. 
    """
    if not self._HasPermission(mbTree, owner):
      return False
    perm = True
    for mbItem in mbTree['subtrees']:
      perm = self._HasPermissionTree(mbItem, owner)
      if not perm:
        break
    return perm

  #--
  def _FindAddPoint(self, mbPath, mbTree):
    """ Find the add point in the menu tree.

        Parameters:
          mbPath    - a '|' separated path string.
          mbTree    - menubar [sub]tree to search
        
        Exceptions:
          KeyError if mbPath already exists or if it cannot be attached.

        Return Value:
          A 2-tuple (tailPath, mbItemParent) where
            tailPath = tail menubar [sub]path of new menu items to be added
            mbItemParent = parent menubar item attach point
          the menu tree add point. 
    """
    label, tailPath = self.Split(mbPath)
    subtrees = mbTree['subtrees']
    for mbItem in subtrees:               # traverse menubar tree
      if mbItem['label'] == label:
        if not tailPath:                  # no more path components
          raise KeyError("Menubar item already exists: %s" % (repr(label)))
        elif mbItem['type'] != gt.MBTypeCascade: # can only add to this type
          raise KeyError(
              "Cannot add menubar item %s: %s is not %s type" \
              % (repr(mbPath), repr(label), repr(gt.MBTypeCascade)))
        else:                             # keep searching
          return self._FindAddPoint(tailPath, mbItem)
    return mbPath, mbTree                 # end of tree branch 

  #--
  def _FindItem(self, mbPath, mbTree):
    """ Find the menubar item specified by the menu path in the menu tree.

        Parameters:
          mbPath    - a '|' separated path string.
          mbTree    - menubar [sub]tree to search
        
        Exceptions:
          KeyError if mbPath is not found.

        Return Value:
          On search succes, return tuple (mbItemParent, index) where
            mbItemParent = menubar parent term holding found menubar item
            index        = Tkinter.Menu IDEX of menu item
    """
    label, tailPath = self.Split(mbPath)
    subtrees = mbTree['subtrees']
    index = 1
    for mbItem in subtrees:               # traverse menubar tree
      if mbItem['label'] == label:
        if not tailPath:                  # found the item
          return (mbTree, index)
        elif mbItem['type'] != gt.MBTypeCascade: # cannot recurse any further
          raise KeyError("Menubar item %s not found" % (repr(mbPath)))
        else:                             # keep searching
          return self._FindItem(tailPath, mbItem)
      index += 1
                                          # end of branch
    raise KeyError("Menubar item %s not found" % (repr(mbPath)))

  #--
  def _ReplaceLastPathComp(self, mbPath, label):
    """ Replace last path compoenent with label. """
    i = mbPath.rfind(gt.MBPS)
    if i <= 0:
      return label
    else:
      return mbPath[:i] + gt.MBPS + label

  #--
  def _PrintTree(self, mbTree=None, indent=0):
    """ Print menu tree to stdout.
    """
    if mbTree == {}: return
    if mbTree is None: mbTree = self.mMenuBarTree

    # print this item
    if mbTree['type'] == 'tearoff':
      print("%*s%s (owner=%s)" % (indent, '', gt.MBPS, mbTree['owner']))
    elif mbTree['type'] == gt.MBTypeCascade:
      print("%*s%s%s (owner=%s, activeDisabled=%s)" \
          % (indent, '', repr(mbTree['label']), 
             gt.MBPS, repr(mbTree['owner']),
             repr(mbTree['activeDisabled'])))
    else:
      print("%*s%s (type=%s, owner=%s, activeDisabled=%s)" \
          % (indent, '', repr(mbTree['label']), 
             repr(mbTree['type']), repr(mbTree['owner']),
             repr(mbTree['activeDisabled'])))

    # print subtrees
    for mbItem in mbTree['subtrees']:
      self._PrintTree(mbItem, indent+2)
 

#-------------------------------------------------------------------------------
# Test Code
#-------------------------------------------------------------------------------

if __name__ == '__main__':
  _Root     = None
  _MenuBar  = None

  def cb1(): print('cb1')

  def cb2(): print('cb2')

  def cbColorMenuBlue():
    print('cbColorMenuBlue')
    colormenu('blue')

  def cbColorMenuBlack():
    print('cbColorMenuBlack')
    colormenu('black')

  def colormenu(color):
    mb = _MenuBar
    for item in mb:
      try:
        mb[item]['foreground'] = color
      except tk.TclError:
        pass

  def cbPrintTree():
    print('cbPrintTree')
    mb = _MenuBar
    mb._PrintTree()

  def cbAddTravel():
    print('cbAddTravel')
    mbmaketraveltree()

  def cbDelTravel():
    print('cbDelTravel')
    mb = _MenuBar
    mb.DelMenuItemsByOwner('travel')

  def cbDisAfricanFalls():
    print('cbDisAfricanFalls')
    mb = _MenuBar
    mb.SetMenuItemStates('falls', 'africa')

  def cbDisSouthAmFalls():
    print('cbDisSouthAmFalls')
    mb = _MenuBar
    mb.SetMenuItemStates('falls', 'southamerica')

  def cbEnAll():
    print('cbEnAll')
    mb = _MenuBar
    mb.SetMenuItemStates('falls', 'nowhere')

  def cbtravel(): print('cbtravel')

  def mbmakeroottree():
    mb = _MenuBar
    mb.AddMenuItem('File|Open...', 'command', owner='root', command=cb1)
    mb.AddMenuItem('File|New...', 'command', owner='root', command=cb1)
    mb.AddMenuItem('File|Save', 'command', owner='root', command=cb1)
    mb.AddMenuItem('File|Save As...', 'command', owner='root', command=cb1)
    seplabel = mb.AddMenuItem('File', 'separator', owner='root')
    mb.AddMenuItem('File|Exit', 'command', owner='root', command=cb1)

    mb.AddMenuItem('View', 'cascade', owner='root', command=cb2)

    mb.AddMenuItem('MenuTest|ColorBlue', 'command', owner='root', 
        command=cbColorMenuBlue)
    mb.AddMenuItem('MenuTest|ColorBlack', 'command', owner='root', 
        command=cbColorMenuBlack)
    mb.AddMenuItem('MenuTest|AddTravel', 'command', owner='root',
        command=cbAddTravel)
    mb.AddMenuItem('MenuTest|DelTravel', 'command', owner='root',
        command=cbDelTravel)
    mb.AddMenuItem('MenuTest|Disable|Falls|African', 'command', owner='root',
        command=cbDisAfricanFalls)
    mb.AddMenuItem('MenuTest|Disable|Falls|South American', 'command', 
        owner='root',
        command=cbDisSouthAmFalls)
    mb.AddMenuItem('MenuTest|Enable All', 'command', 
        owner='root',
        command=cbEnAll)
    mb.AddMenuItem('MenuTest', 'separator', owner='root')
    mb.AddMenuItem('MenuTest|PrintTree', 'command', owner='root',
        command=cbPrintTree)

    mb.AddMenuItem('Help|Fusion', 'command', owner='root', command=cb2)
    mb.AddMenuItem('Help', 'separator', owner='root')
    mb.AddMenuItem('Help|About', 'command', owner='root', command=cb2)

  def mbmaketraveltree():
    mb = _MenuBar
    mb.InsMenuItem('Help', 'Falls|Niagara...', 'command',
        owner='travel', command=cbtravel,
        disabledStates={'falls':['northamerica']})
    mb.AddMenuItem('Falls|Angel...', 'command',
        owner='travel', command=cbtravel,
        disabledStates={'falls':['southamerica']})
    mb.AddMenuItem('Falls', 'separator', owner='travel')
    mb.AddMenuItem('Falls|Africa', 'cascade', owner='travel')
    mb.AddMenuItem('Falls|Africa|Victoria', 'checkbutton',
        owner='travel', command=cbtravel,
        disabledStates={'falls':['africa']})
    mb.AddMenuItem('Falls|Africa|Augrabies', 'radiobutton',
        owner='travel', command=cbtravel,
        disabledStates={'falls':['africa']})
                                                                                
    mb.InsMenuItem('Help', 'Travel', 'command', owner='travel', 
        command=cbtravel)
                                                                                
    mb.AppMenuItem('Falls', 'Lakes|Erie', 'command',
      owner='travel', command=cbtravel)
    mb.AddMenuItem('Lakes|Ontario', 'command', owner='travel', 
        command=cbtravel)
    mb.AddMenuItem('Lakes|Superior', 'command', owner='travel', 
        command=cbtravel)
    mb.AddMenuItem('Lakes|Huron', 'command', owner='travel', command=cbtravel)
    mb.AddMenuItem('Lakes|Michigan', 'command', owner='travel', 
        command=cbtravel)
                                                                                
    seplabel = mb.AppMenuItem('File|Save As...', '', 'separator',
        owner='travel')
    mb.AppMenuItem(seplabel, 'Travel Profile', 'command',
      owner='travel', command=cbtravel)

  def main():
    """ GuiMenuBar Test Main """
    global _Root, _MenuBar
    _Root = tk.Tk()
    _MenuBar = GuiMenuBar(_Root)
    mbmakeroottree()
    _Root.mainloop()

  # run test
  main()
