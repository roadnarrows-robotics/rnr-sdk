################################################################################
#
# vBrain.py
#

""" Virtual Brain Virtual Base Class.

We're not computers, Sebastian, we're physical.

The Virtual Brain virtual base class defines the required brain interface
to control a vRobot (and a physical robot).

A Brain in a jar

A vBrain can be written such that it does not need a body (robot) if
desired. Fusion will take care of it.

Author: Robin D. Knight
Email:  robin.knight@roadnarrowsrobotics.com
URL:    http://www.roadnarrowsrobotics.com
Date:   2005.12.29

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

import Fusion.Utils.Enum as enum
import Fusion.Core.Gluon as Gluon

#-------------------------------------------------------------------------------
# Global Data
#-------------------------------------------------------------------------------

#-------------------------------------------------------------------------------
# CLASS: vBrain
#-------------------------------------------------------------------------------
class vBrain(Gluon.GluonServer):
  """ Virtual Brain Virtual Base Class. """

  #--
  def __init__(self, serverId='vBrain', client=None, menubarList=None, 
                     debuglevel=0, debugfout=None):
    """ Initialize vBrain base instance.

        Parameters:
          serverId    - this Gluon server id
          client      - Gluon client
          menubarList - server menubar list (GluonMenuBarList object)
          debuglevel  - none to all [0, 5] (default: none)
          debugfout   - opened debug output file (default: stdout)
    """
    Gluon.GluonServer.__init__(self, serverId=serverId, client=client,
                                     menubarList=menubarList,
                                     debuglevel=debuglevel, debugfout=debugfout)

    # vBrain initialization
    self.vBrainInit()

  #--
  def vBrainInit(self):
    """ One-time vRobot initialization during object instantiation. """
    # Bootstrap data initialization (in case of init interdependencies)
    self.mBrum = {}
    self.mPeerRobotCallbacks = {}

    # Now initialize
    self.IniInit()        # ini initialization
    self.PeerRobotInit()  # initialize i/f with vRobot peer
    self.BrumInit()       # the brum initialization


  # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  # vBrain Attribute Member Functions
  # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

  #--
  def IsType(self):
    """ Returns the type of this brain.

        Return Value:
          The brain MIME type string is in the format:
            brain/subtype
    """
    return 'brain/unknown'

  #--
  def HasName(self):
    """ Returns the short brain name(s) string.

        Return Value:
          The brain name(s) string which may include either or both
          the vBrain and physical brain names.
    """
    return 'brain'

  #--
  def IsVersion(self):
    """ Returns the vBrain version(s) string.

        Return Value:
          The brain version(s) string.
    """
    return '0.0'

  #--
  def HasDesc(self):
    """ Returns a short description of this brain.

        Return Value:
          Multi-line description string.
    """
    sDesc = """No description available."""
    return sDesc


  # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  # vBrain Ini (Re)Initializatio Member Functions
  # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

  #--
  def IniInit(self):
    """ (Re)Initialize vBrain from the parsed 'ini'configuration.
    
        This function is implementation specific.

        Return Value:
          None
    """
    pass


  # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  # vBrain to vRobot Peer Member Functions
  # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

  #--
  def GSRegisterPeer(self, peer):
    """ Register vRobot Gluon server peer to this vBrain.

        Parameters:
          peer  - (derived) GluonServer object

        Return Value:
          Returns True of peer is compatible with this server.
          Else returns False.
    """
    self.mPeer = peer
    return self.PeerRobotEstablish()

  #--
  def GSUnregisterPeer(self):
    """ Unregister the vRobot Gluon server from this vBrain.

        Return Value:
          None
    """
    self.PeerRobotTerminate()
    self.mPeer = None

  #--
  def PeerRobotInit(self):
    """ Initialize vRobot peer callback interface.

        mPeerRobotCallbacks Format:
          {'callbackname': callback, ...}

        Return Value:
          None
    """
    self.mPeerRobotCallbacks = {}
    self.PeerRobotNull()

  #--
  def PeerRobotNull(self):
    """ Set any vBrain peer data to the null set.

        Implementation specific.

        Return Value:
          None
    """
    pass

  #--
  def PeerRobotEstablish(self):
    """ Establish the vBrain to vRobot peer interface when the vRobot
        peer registers with this vBrain.

        Implementation specific.

        Return Value:
          True if vRobot peer is compatible with this vBrain and the
          vRobot has sufficient resources. Else return False.
    """
    return True

  #--
  def PeerRobotTerminate(self):
    """ Terminate the vBrain to vRobot peer interface when the vRobot
        peer unregisters with this vBrain.

        Default action is to set all callback interfaces to the 
        PeerRobotCbNoOp callback.

        Return Value:
          None
    """
    self.PeerRobotNull()

  #--
  def PeerRobotCbNoOp(self, *args, **kwargs):
    """ A NoOp vRobot peer callback. All agruments are ignored and no
        true peer callback is performed.
        
        Return Value:
          An empty dictionary {}.
        
        """
    return {}


  # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  # vBrain (Cere)Brum Member Functions
  # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

  #--
  def BrumInit(self):
    """ Initialize the vBrain's high-level brum innate data and learned 
        states.

        The (cere)brum integrates information from all of the sensory data,
        initiates effector functions, controls emotions and holds memory and
        thought processes. The brum is shared between the think and act
        processes.

        Return Value:
          None
    """
    self.mBrum = {}     # create an empty brain
    self.BrumNull()     # and null it out

  #--
  def BrumNull(self):
    """ Set the vBrains's high-level states to the null set.

        Return Value:
          Returns new current brum state (implementation specific). 
    """
    return self.mBrum

  #--
  def BrumGet(self):
    """ Get the current brain high-level state.

        Return Value:
          Current brum state (implementation specific).
    """
    return self.mBrum

  #--
  def BrumSet(self, **kwStates):
    """ Set new vBrain high-level state(s). The old states are overwritten.

        Parameters:
          kwStates  - argument list of keyword=value states(s) 
                      (implementation specific)
        
        Return Value:
          Returns new current brum state. 
    """
    for k,v in kwStates.iteritems():
      self.mBrum[k] = v
    return self.mBrum


  # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  # vBrain State Member Function
  # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

  #--
  def GetBrainState(self):
    """ Get the brain's current server state. Synonym for GSGetServerState()

        Return Value:
          Returns current state EServerState
    """
    return self.GSGetServerState()

  #--
  def SetBrainState(self, newState):
    """ Set the brain's new server state. Synonym for GSSetServerState()

        Parameters:
          newState  - new brain server state EServerState

        Return Value:
          Returns old brain server state.
    """
    return self.GSSetServerState(newState)
