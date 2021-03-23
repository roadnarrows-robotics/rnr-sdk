################################################################################
#
# Gluon.py
#

""" The Gluon "Glue" Module between Fusion, vRobot, and vBrain Objects.

The Gluon Module provides the common "glue" between the vRobot and
vBrain servers and the Fusion client.

Author: Robin D. Knight
Email:  robin.knight@roadnarrowsrobotics.com
URL:    http://www.roadnarrowsrobotics.com
Date:   2005.11.20

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
import threading as thread

import Fusion.Utils.IniParser as IniParser
import Fusion.Utils.Enum as enum
import Fusion.Gui.GuiTypes as gt

if __debug__: import Fusion.Utils.PyDebug as PyDebug

#-------------------------------------------------------------------------------
# Global Data
#-------------------------------------------------------------------------------

#
# Server State Enumeration
#   Undef       undefined pre-server existence (s0)
#   NotLoaded   server is not loaded (uninitialized, unknown state)
#   NotReady    loaded, but not ready
#   Ready       ready to run
#   Paused      paused (suspended)
#   Stepping    stepping one cycle
#   Running     running
#   Errored     fatal errored condition (requires re-initialization)
#
EServerState = enum.Enum(
    'Undef NotLoaded NotReady Ready Paused Stepping Running Errored')

#
# Server Executive Primitives
#   EStop       emergency stop
#   Load        load the server
#   Start       start the server running
#   Resume      resume the server running after being suspended
#   Suspend     suspend (pause) the server from running
#   Step        step the server one execution cycle
#   Stop        stop the server and 'rewind' to ready
#   Unload      unload the server
#
EServerExec = enum.Enum('Load EStop Start Resume Suspend Step Stop Unload')

#
# Invalid States for Executive Primatives 
#
ServerInvalidExecStateTbl = {
  EServerExec.Load:     [EServerState.Undef] + \
                         range(EServerState.NotReady, EServerState.numof()),

  EServerExec.EStop:    [EServerState.Undef, EServerState.NotLoaded],

  EServerExec.Start:    [EServerState.Undef, EServerState.NotLoaded, 
                         EServerState.NotReady, EServerState.Paused,
                         EServerState.Stepping, EServerState.Running,
                         EServerState.Errored],

  EServerExec.Resume:   [EServerState.Undef, EServerState.NotLoaded,
                         EServerState.NotReady, EServerState.Ready,
                         EServerState.Stepping, EServerState.Running,
                         EServerState.Errored],

  EServerExec.Suspend:  [EServerState.Undef, EServerState.NotLoaded,
                         EServerState.NotReady, EServerState.Ready,
                         EServerState.Paused, EServerState.Errored],

  EServerExec.Step:     [EServerState.Undef, EServerState.NotLoaded,
                         EServerState.NotReady, EServerState.Stepping,
                         EServerState.Errored],

  EServerExec.Stop:     [EServerState.Undef, EServerState.NotLoaded,
                         EServerState.NotReady, EServerState.Ready],

  EServerExec.Unload:   [EServerState.Undef, EServerState.NotLoaded]
}


#-------------------------------------------------------------------------------
# CLASS: GluonServer
#-------------------------------------------------------------------------------
class GluonServer:
  """ Gluon Server Virtual Class.

      vRobot and vBrain are Gluon Server objects.
  """

  #--
  def __init__(self, serverId='GluonServer', client=None, menubarList=None,
                     toolbarList=None, debuglevel=0, debugfout=None):
    """ Initialize Gluon Server instance.

        Parameters:
          serverId    - Gluon server id string
          client      - Gluon client 
          menubarList - server GluonMenuBarList object
          toolbarList - server GLuonToolBarList object
          debuglevel  - none to all [0, 5] (default: none)
          debugfout   - opened debug output file (default: stdout)
    """
    if __debug__:
      self.mDbg = PyDebug.PyDebug(preface=serverId,
                                  debuglevel=debuglevel, fout=debugfout)

    self.mServerId    = serverId            # Gluon server id
    self.mClient      = None                # Gluon client
    self.mPeer        = None                # Gluon server peer
    self.mMenuBarList = menubarList         # server menubar list
    self.mToolBarList = toolbarList         # server toolbar list
    self.mIniDD       = None                # server 'ini' definition dictionary
    self.mServerState = EServerState.NotLoaded # starting server state
    self.mLocalIni    = None                # local ini data. only used if
                                            # no attached server
    self.mServerSema  = thread.Semaphore()  # server lock/unlock semaphore

    if client:
      self.GSRegisterClient(client)
    else:
      self.GSUnregisterClient()

  #--
  def GSLock(self):
    """ Lock this server to have sole ownership. """
    self.mServerSema.acquire()

  #--
  def GSUnlock(self):
    """ Unlock this server to share. """
    self.mServerSema.release()

  #--
  def GSGetServerId(self):
    """ Get this server's identity.

        Return Value:
          The unique server id string
    """
    return self.mServerId

  #--
  def GSRegisterClient(self, client):
    """ Register Gluon client to this server.

        Parameters:
          client  - (derived) GluonClient object

        Return Value:
          None
    """
    self.mClient = client

    # synchronize ini configurations, the client ini takes precedence
    if self.mLocalIni:
      if __debug__: self.mDbg.d2print("Synchronizing configurations")
      self.GSReportNormalStatus("Synchronizing configurations")
      clientIni = self.mClient.GCGetIni()
      numMerges = clientIni.IniSync(self.mLocalIni)
      del self.mLocalIni
      self.mLocalIni = None

    # report current state
    self.mClient.GCReportServerStateChanged(self.mServerId, self.mServerState)

  #--
  def GSUnregisterClient(self):
    """ Unregister the Gluon client.

        Return Value:
          None
    """
    self.mClient = None

  #--
  def GSRegisterPeer(self, peer):
    """ Register Gluon server peer to this server.

        Currently, only one peer is supported.

        Parameters:
          peer  - (derived) GluonServer object

        Return Value:
          Returns True of peer is compatible with this server.
          Else returns False.
    """
    self.mPeer = peer
    return True

  #--
  def GSUnregisterPeer(self):
    """ Unregister the Gluon server peer.

        Return Value:
          None
    """
    self.mPeer = None

  #--
  def GSGetIni(self):
    """ Get the registered Gluon client's parsed ini data.
        Note: if the server is running without a client, get a local
              ini object to guarantee an ini.

        Return Value:
          Returns the parsed ConfigParser object if client registered.
          Else returns Local Ini.
    """
    if self.mClient:
      return self.mClient.GCGetIni()
    elif not self.mLocalIni:
      self.mLocalIni = IniParser.IniParser() # local ini data
    return self.mLocalIni

  #--
  def GSGuiGetParent(self):
    """ Get the parent GUI object from the registered Gluon client.

        Return Value:
          Returns the GUI object if client registered.
          Else returns None.
    """
    if self.mClient:
      return self.mClient.GCGuiGetParent()
    else:
      return None

  def GSGuiRaiseParent(self):
    """ Raise GUI parent object to front of display.

        Return Value:
          None
    """
    if self.mClient:
      return self.mClient.GCGuiRaiseParent()

  #--
  def GSGuiWinGetOptions(self, winId):
    """ Retrieves from the client any saved server Gui window options,
        plus the core GuiWin options.

        Parameters:
          winId     - Gui window unique id

        Return Value:
          **options - list of previous configuration options (if any),
                      plus GuiWin core options.
    """
    if self.mClient:
      return self.mClient.GCGuiWinGetOptions(self.mServerId, winId)
    else:
      return {}

  #--
  def GSGuiWinRegister(self, winId, win):
    """ Register server GUI window with the client.

        Parameters:
          winId     - Gui window unique id. None if no id.
          win       - Gui window object

        Return Value:
          None
    """
    if self.mClient:
      self.mClient.GCGuiWinRegister(self.mServerId, winId, win)

  #--
  def GSGuiWinUnregister(self, winId, win, **saveOpts):
    """ Unregister server GUI window with the client.

        Parameters:
          winId     - Gui window unique id. None if no id.
          win       - Gui window object
          saveOpts  - window keyword configuration options to save 

        Return Value:
          None
    """
    if self.mClient:
      self.mClient.GCGuiWinUnregister(self.mServerId, winId, win, **saveOpts)

  #--
  def GSGuiWinStart(self, winId, callobj, *args, **kwargs):
    """ Start a server child window. Any ini saved configuration is 
        retrieved and added to the keywoard arguments. The started 
        window is automatically registered with the client.

        Parameters:
          winId     - Gui window unique id. None if no id.
          callobj   - callable object to start the window. Must return
                      Gui window object.
          args      - arguments passed to callobj()
          kwargs    - keyword arguments passed to callobj().

        Return Value:
          Gui window object
    """
    if self.mClient:
      return self.mClient.GCGuiWinStart(self.mServerId, winId,
                                        callobj, *args, **kwargs)
    else:
      return None

  #--
  def GSGuiWinUpdate(self, winId, *args, **kwargs):
    """ Make callback to server child window update function.

        Parameters:
          winId     - gui window unique id.
          *args     - arguments to update callback
          *kwargs   - keyword arguments to update callback

        Return Value:
          None
    """
    if self.mClient:
      self.mClient.GCGuiWinUpdate(self.mServerId, winId, *args, **kwargs)

  #--
  def GSGetServerMenuBarList(self):
    """ Get this server's list of menubar items to be added to the GUI.

        Return Value:
          Returns the server's GluonMenuBarList object.
    """
    return self.mMenuBarList

  #--
  def GSSetServerMenuBarList(self, menubarList):
    """ Get this server's list of menubar items to be added to the GUI.

        Parameters:
          menubarList - list of menubar items (GluonMenuBarList object)

        Return Value:
          None
    """
    self.mMenuBarList = menubarList

  #--
  def GSGetServerToolBarList(self):
    """ Get this server's list of toolbar items to be added to the GUI.

        Return Value:
          Returns the server's GluonToolBarList object.
    """
    return self.mToolBarList

  #--
  def GSSetServerToolBarList(self, toolbarList):
    """ Get this server's list of toolbar items to be added to the GUI.

        Parameters:
          toolbarList - list of toolbar items (GluonToolBarList object)

        Return Value:
          None
    """
    self.mToolBarList = toolbarList

  #--
  def GSGetServerIniDD(self):
    """ Get this server's dictionary of 'ini' definitions.'
        See FusionIniDesc.py for expected format.

        Return Value:
          Returns the server's 'ini' definition dictionary.
    """
    return self.mIniDD

  #--
  def GSSetServerIniDD(self, iniDD):
    """ Set this server's dictionary of 'ini' definitions.'
        See FusionIniDesc.py for expected format.

        Parameters:
          Dictionary of 'ini' definitions. 
    """
    self.mIniDD = iniDD

  #--
  def GSGetServerState(self):
    """ Get this server's current state.

        Return Value:
          Returns current server state EServerState
    """
    return self.mServerState

  #--
  def GSSetServerState(self, newState):
    """ Set this server's current state.

        Parameters:
          newState  - new EServerState

        Return Value:
          Returns previous server state EServerState
    """
    oldState = self.mServerState
    self.mServerState = newState
    if self.mClient:
      self.mClient.GCReportServerStateChanged(self.mServerId, self.mServerState)
    if __debug__: self.mDbg.d1print("New server state: %s(%d)" % \
        (EServerState.name(self.mServerState), self.mServerState))
    return oldState

  #--
  def GSIsValidExecState(self, serverExec):
    """ Validate that the server is in the correct state for the given
        play executive primitive. Reports error if invalid.

        Parameters:
          serverExec  - EServerExec value

        Return Value:
          True if the server state is valid for the executive. 
          False otherwise.
    """
    if self.mServerState in ServerInvalidExecStateTbl[serverExec]:
      self.GSReportErrorStatus("Invalid state for %s executive: state = %s" % \
          (repr(EServerExec.name(serverExec)),
          repr(EServerState.name(self.mServerState))))
      return False
    else:
      return True

  #--
  def GSReportNormalStatus(self, msg):
    """ Report a normal status message to registered Gluon client.

        Parameters:
          msg  - message string

        Return Value:
          None
    """
    if self.mClient:
      self.mClient.GCReportNormalStatus(self.mServerId, msg)
    else:
      print("%s: %s" % (self.mServerId, msg))

  #--
  def GSReportErrorStatus(self, emsg):
    """ Report an error status message to registered Gluon client.

        Parameters:
          emsg  - error message string

        Return Value:
          None
    """
    if self.mClient:
      self.mClient.GCReportErrorStatus(self.mServerId, emsg)
    else:
      print("error: %s: %s" % (self.mServerId, emsg))

  def GSSetDebugLevel(self, debuglevel, debugfout=None):
    """ Set server debugging level.

        Parameters:
          debuglevel  - none to all [0, 5]
          debugfout   - opened debug output file
        
        Return Value:
          None
    """
    if __debug__: self.mDbg.On(debuglevel, debugfout)
    pass # keep this line


  # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  # Derived Gluon Server Play (Required) Executive Primitives
  # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

  def ExecEStop(self):
    """ Execute server emergency stop operation. """
    raise NotImplementedError("%s.ExecEStop" % (self.__class__.__name__))
    
  def ExecLoad(self):
    """ Execute load (initialize) server operation. """
    raise NotImplementedError("%s.ExecLoad" % (self.__class__.__name__))
    
  def ExecStart(self):
    """ Execute run at the start server operation. """
    raise NotImplementedError("%s.ExecStart" % (self.__class__.__name__))
    
  def ExecSuspend(self):
    """ Execute suspend server operation. """
    raise NotImplementedError("%s.ExecSuspend" % (self.__class__.__name__))
    
  def ExecResume(self):
    """ Execute resume server operation. """
    raise NotImplementedError("%s.ExecResume" % (self.__class__.__name__))
    
  def ExecStep(self):
    """ Execute single step operation. """
    raise NotImplementedError("%s.ExecStep" % (self.__class__.__name__))
    
  def ExecStop(self):
    """ Execute stop server operation. """
    raise NotImplementedError("%s.ExecStop" % (self.__class__.__name__))
    
  def ExecUnload(self):
    """ Execute unload (de-initialize) server operation. """
    raise NotImplementedError("%s.ExecUnload" % (self.__class__.__name__))
    


#-------------------------------------------------------------------------------
# CLASS: GluonClient
#-------------------------------------------------------------------------------
class GluonClient:
  """ Gluon Client Virtual Class.

      Fusion is a Gluon Client object.
  """
  #--
  def __init__(self, clientId='GluonClient', debuglevel=0, debugfout=None):
    """ Initialize instance.

        Parameters:
          clientId    - Gluon client id string
          debuglevel  - none to all [0, 5] (default: none)
          debugfout   - opened debug output file (default: stdout)
    """
    if __debug__:
      self.mDbg = PyDebug.PyDebug(preface=clientId,
                                  debuglevel=debuglevel, fout=debugfout)

    self.mClientId      = clientId  # Gluon client id
    self.mServers       = {}        # Gluon servers to this client
    self.mIni           = IniParser.IniParser() # ini data
    self.mClientSema    = thread.Semaphore()  # client lock/unlock semaphore

  #--
  def GCLock(self):
    """ Lock this client to have sole ownership. """
    self.mClientSema.acquire()

  #--
  def GCUnlock(self):
    """ Unlock this client to share. """
    self.mClientSema.release()

  #--
  def GCGetClientId(self):
    """ Get this client's identity.

        Return Value:
          The unique client id string
    """
    return self.mClientId

  #--
  def GCRegisterServer(self, serverId, server):
    """ Register Gluon server to this client.

        Parameters:
          serverId  - server identification
          server    - (derived) GluonServer object

        Return Value:
          None
    """
    self.mServers[serverId] = server

  #--
  def GCUnregisterServer(self, serverId):
    """ Unregister the Gluon server.

        Parameters:
          serverId  - server identification

        Return Value:
          None
    """
    if self.mServers.has_key(serverId):
      del self.mServers[serverId]

  #--
  def GCGetServer(self, serverId):
    """ Gets the Gluon server object with the given id.

        Parameters:
          serverId  - server identification

        Return Value:
          On success, Gluon server object. 
          On failure, None
    """
    if self.mServers.has_key(serverId):
      return self.mServers[serverId]
    else:
      return None

  #--
  def GCGetIni(self):
    """ Get the parsed ini data.

        Return Value:
          Returns the parsed ConfigParser object.
    """
    return self.mIni

  #--
  def GCGuiGetParent(self):
    """ Get the parent GUI object.

        The derived GluonClient should override to provide the real
        object.

        Return Value:
          Returns the GUI object.
    """
    return None

  def GCGuiRaiseParent(self):
    """ Raise GUI parent object to front of display.

        The derived GluonClient should override to do the real work.

        Return Value:
          None
    """
    pass

  #--
  def GCGuiWinGetOptions(self, serverId, winId):
    """ Retrieves any saved options for the server Gui window,
        plus the core GuiWin options.

        Parameters:
          serverId  - server identification
          winId     - Gui window unique id

        Return Value:
          **options - list of previous configuration options (if any),
                      plus GuiWin core options.
    """
    return {}

  #--
  def GCGuiWinRegister(self, serverId, winId, win):
    """ Register server GUI window with the client.

        Parameters:
          serverId  - server identification
          winId     - Gui window unique id. None if no id.
          win       - Gui window object

        Return Value:
          None
    """
    pass

  #--
  def GCGuiWinUnregister(self, serverId, winId, win, **saveOpts):
    """ Unregister server GUI window with the client.

        Parameters:
          serverId  - server identification
          winId     - Gui window unique id. None if no id.
          win       - Gui window object
          saveOpts  - window keyword configuration options to save 

        Return Value:
          None
    """
    pass

  #--
  def GCGuiWinStart(self, serverId, winId, callobj, *args, **kwargs):
    """ Start a server child window. Any ini saved configuration is 
        retrieved and added to the keyword arguments. The started 
        window is automatically registered with the client.

        Parameters:
          serverId  - server identification
          winId     - Gui window unique id. None if no id.
          callobj   - callable object to start the window. Must return
                      Gui window object.
          args      - arguments passed to callobj()
          kwargs    - keyword arguments passed to callobj().

        Return Value:
          Gui window object
    """
    return None

  #--
  def GCGuiWinUpdate(self, serverId, winId, *args, **kwargs):
    """ Make callback to server child window update function.

        Parameters:
          serverId  - server identification
          winId     - gui window unique id.
          *args     - arguments to update callback
          *kwargs   - keyword arguments to update callback

        Return Value:
          None
    """
    pass

  #--
  def GCReportServerStateChanged(self, serverId, state):
    """ Effect any client changes to the event that the registered Gluon
        server's state has changed. 
        
        The derived GluonClient should override to keep track of state changes.

        Parameters:
          serverId  - registered server id string
          state     - new server EServerState

        Return Value:
          None
    """
    pass

  #--
  def GCReportNormalStatus(self, serverId, msg):
    """ Received a normal status message from registered Gluon server.
        
        The derived GluonClient should override if simple text output to
        stdout is not desired.

        Parameters:
          serverId  - registered server id string
          msg       - server message string

        Return Value:
          None
    """
    print("%s: %s" % (serverId, msg))

  #--
  def GCReportErrorStatus(self, serverId, emsg):
    """ Received an error status message from registered Gluon server.
        
        The derived GluonClient should override if simple text output to
        stdout is not desired.

        Parameters:
          serverId  - registered server id string
          emsg      - server error message string

        Return Value:
          None
    """
    print("error: %s: %s" % (serverId, emsg))

  #--
  def GCSetDebugLevel(self, debuglevel, debugfout=None):
    """ Set client debugging level.

        Parameters:
          debuglevel  - none to all [0, 5]
          debugfout   - opened debug output file
        
        Return Value:
          None
    """
    if __debug__: self.mDbg.On(debuglevel, debugfout)
    pass  # keep this line


#-------------------------------------------------------------------------------
# CLASS: GluonMenuBarList
#-------------------------------------------------------------------------------
class GluonMenuBarList:
  """ Gluon MenuBar List Container Class.

      Provides structure and access control of to a list of menubar items.
      Although menu item types match the Tkinter.Menu types, this class
      is actually independent of any GUI.
  """

  #--
  def __init__(self):
    """ Initialize menubar list instance. """
    self.mList = []   # order is important, so don't use a dictionary

  def __iter__(self):
    """ menubarPath,menuItem Iterator. """
    return self.mList.__iter__()

  def add(self, mbPath, itemType, callback=None, disabledStates={}): 
    """ Add menubar item to the list. The new item is validated prior
        to adding. A ValueError is raised on any input value error.

        Parameters:
          mbPath          - unique menubar path. Path components are
                            separated by the '|' character.
          itemType        - menubar item type: One of:
                              MBTypeCascade, MBTypeCommand, MBTypeCheckButton,
                              MBTypeRadioButton, MBTypeSeparator
          callback        - menu item callback function
          disabledStates  - dictionary of state spaces. Each space has a 
                            list of disabled states whose elements can be 
                            any comparable object.

        Return Value:
          None
    """
    if not mbPath:
      raise ValueError("mbPath string required")
    elif type(mbPath) != str:
      raise ValueError("mbPath not a string: %s" % (repr(mbPath)))
    if itemType == gt.MBTypeSeparator:
      mbPath = gt.MBMakeSepLabel(mbPath)
    if self.getitem(mbPath):
      raise ValueError("Duplicate mbPath string: %s" % (mbPath))
    if itemType == gt.MBTypeCascade:
      callback = None
    elif itemType == gt.MBTypeCommand or \
         itemType == gt.MBTypeCheckButton or \
         itemType == gt.MBTypeRadioButton:
      if not callback:
        raise ValueError("callback function required")
      elif not callable(callback):
        raise ValueError("callback not a function")
    elif itemType == gt.MBTypeSeparator:
      callback = None
      disabledStates={}
    else:
      raise ValueError("Unknown itemType: %s" % (repr(itemType)))
    self.mList += [{'path':mbPath, 'type':itemType, 'command':callback, 
                   'disabledStates':disabledStates}]

  def getitem(self, mbPath):
    """ Get the menubar item identified by the menubar path.

        Parameters:
          mbPath          - unique menubar path. Path components are
                            separated by the '|' character.

        Return Value:
          Return menubar item if found, else returns None.
    """
    for mbItem in self.mList:
      if mbItem['path'] == mbPath:
        return mbItem
    return None


#-------------------------------------------------------------------------------
# CLASS: GluonToolBarList
#-------------------------------------------------------------------------------
class GluonToolBarList:
  """ Gluon ToolBar List Container Class.

      Provides structure and access control of to a list of toolbar items.
      GluonToolBarList is independent of an GUI implementation.
  """

  #--
  def __init__(self):
    """ Initialize toolbar list instance. """
    self.mList = []   # order is important, so don't use a dictionary

  def __iter__(self):
    """ toolbarPath,toolItem Iterator. """
    return self.mList.__iter__()

  def add(self, label, callback, **options):
    """ Add toolbar item to the list. The new item is validated prior
        to adding. A ValueError is raised on any input value error.

        Parameters:
          label           - button unique text label
          callback        - button callback function
          options         - any keyword=value arguments passed to the 
                            underlining ToolBar implementation. 

        Return Value:
          None
    """
    if not label:
      raise ValueError("label string required")
    elif type(label) != str:
      raise ValueError("label not a string: %s" % (repr(label)))
    elif self.getitem(label):
      raise ValueError("Duplicate label string: %s" % (label))
    if not callback:
      raise ValueError("callback function required")
    elif not callable(callback):
      raise ValueError("callback not a function")
    self.mList += [{'type':'button', 'label':label, 'command':callback,
                    'options':options}]

  def addsep(self, space):
    """ Add seperator space to list.
        Parameters:
          space - amount of space ' '
    """
    self.mList += [{'type':'sep', 'space':space}]

  def getitem(self, label):
    """ Get the toolbar item identified by the toolbar label.

        Parameters:
          label           - button unique text label

        Return Value:
          Return toolbar item if found, else returns None.
    """
    try:
      i = self.mList.index(label)
      return self.mList[i]
    except ValueError:
      return None
