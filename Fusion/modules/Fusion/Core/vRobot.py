################################################################################
#
# vRobot.py
#

""" Virtual Robot Virtual Base Module.

Klaatu barada nikto!

Virtual Robot virtual base module defines the required physical/simulated
robot interface.

Author: Robin D. Knight
Email:  robin.knight@roadnarrowsrobotics.com
URL:    http://www.roadnarrowsrobotics.com
Date:   2005.11.08

Copyright (C) 2005, 2006  RoadNarrows LLC.
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
# CLASS: vRobot
#-------------------------------------------------------------------------------
class vRobot(Gluon.GluonServer):
  """ Virtual Robot Virtual Base Class. """

  #--
  def __init__(self, serverId='vRobot', client=None, 
                     menubarList=None, toolbarList=None,
                     debuglevel=0, debugfout=None):
    """ Initialize vRobot base instance.

        Parameters:
          serverId    - this Gluon server id
          client      - Gluon client
          menubarList - server menubar list (GluonMenuBarList object)
          toolbarList - server GLuonToolBarList object
          debuglevel  - none to all [0, 5] (default: none)
          debugfout   - opened debug output file (default: stdout)
    """
    # server initialization
    Gluon.GluonServer.__init__(self, serverId=serverId, client=client,
                                     menubarList=menubarList,
                                     toolbarList=toolbarList,
                                     debuglevel=debuglevel, debugfout=debugfout)

    # vRobot initialization
    self.vRobotInit()

  #--
  def vRobotInit(self):
    """ One-time vRobot initialization during object instantiation. """
    # Bootstrap data initialization (in case of init interdependencies)
    self.mCommIsUp = False  # communication channel is not up
    self.mShadow = {}
    self.mBellumGoalSet = {}

    # Now initialize
    self.IniInit()          # ini initialization
    self.ShadowInit()       # the shadow initialization
    self.BellumInit()       # the bellum initialization


  # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  # vRobot Attribute Member Functions
  # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

  #--
  def IsType(self):
    """ Returns the type of this robot.

        Return Value:
          The robot MIME type string is in the format: robot/subtype
    """
    return 'robot/unknown'

  #--
  def HasName(self):
    """ Returns the short robot name(s) string.

        Return Value:
          The robot name(s) string which may include either or both
          the vRobot and physical robot names.
    """
    return 'robot'

  #--
  def IsVersion(self):
    """ Returns the vRobot version(s) string.

        Return Value:
          The vRobot version string(s).
    """
    return '0.0'

  #--
  def IsRobotVersion(self):
    """ Returns the physical robot version(s) string.

        Return Value:
          Version string.
    """
    return 'unknown'

  #--
  def HasDesc(self):
    """ Returns a short description of of this vRobot.

        Return Value:
          Multi-line description string.
    """
    sDesc = """No description available."""
    return sDesc

  #--
  def HasSensorTypes(self):
    """ Returns the dictionary of the sensors and their corresponding
        properties that are available and supported by this vRobot.
        The dictionary is keyed by 'sensorId', which is vRobot unique.
    
        Return Value:
          Dictionary of sensor id's and properties in the format:
            {sensorId:{'mimetype':<type>...}, ... }
    """
    return {}

  #--
  def HasEffectorTypes(self):
    """ Returns the dictionary of the effectors and their corresponding
        properties that are available and supported by this vRobot.
        The dictionary is keyed by 'effectorId', which is vRobot unique.
    
        Return Value:
          Dictionary of effector id's and properties in the format:
            {effectorId:{'mimetype':<type>...}, ... }
    """
    return {}

  #--
  def HasPhysicalProperties(self):
    """ Returns the dictionary of physical properties of this vRobot.
        Typical properties are weight, diameter, wheel base(s), payload
        capacities, and broad capabilites.
    
        Return Value:
          Dictionary of physical properties in the format:
            {property:{'val':<val>, 'units':<units>...}, ... }
    """
    return {}

  #--
  def HasShadowGroups(self):
    """ Returns the maximal list of sensor shadow groups (keys) supported
        by this vRobot. The vRobot may or may not update these groups
        depending the the options selected and the configuration of the
        physical robot.
    
        Return Value:
          List of shadow group Ids.
    """
    groups = []
    for groupId in self.mShadow.iterkeys():
      groups.append(groupId)
    return groups

  #--
  def HasBellumGoalControls(self):
    """ Returns the maximal list of goal controls (kwGoals) supported
        by this vRobot. Goal controls are highly implementation
        specific.
    
        This function is implementation specific.

        Return Value:
          List of goal controls.
    """
    pass


  # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  # vRobot Ini (Re)Initializatio Member Functions
  # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

  #--
  def IniInit(self):
    """ (Re)Initialize vRobot from the parsed 'ini'configuration.
    
        This function is implementation specific.

        Return Value:
          None
    """
    pass


  # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  # vRobot Sensor Shadow Member Functions
  # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

  def ShadowInit(self):
    """ Initialize the shadowed data.

        The Shadow captures the sensory states of the (simulated) robot
        plus any derived states into a dictionary of data. This data can
        be readily accessed by the vRobot's reactive subsystem and by the
        upper layer vBrain functions.

        This function is implementation specific.

        Return Value:
          None.
    """
    self.GSLock()
    self.mShadow = {}
    self.GSUnlock()

  #--
  def ShadowGet(self, groupId='all'):
    """ Get the current shadowed data group(s). Shadowed data are any
        enabled sensor reading(s) of the specified type plus any 
        derived state data.

        Parameters:
          groupId  - shadow group Id. (default: 'all')
        
        Return Value:
          Returns dictionary of current read sensor values.
    """
    self.GSLock()
    if groupId == 'all':
      d = self.mShadow
    else:
      d = self.mShadow[groupId]
    self.GSUnlock()
    return d

  #--
  def ShadowUpdate(self, groupId='all'):
    """ Force an update of the shadow data group(s) from the current
        states of the robot. The vRobot typically updates this in it's
        executio cycle.  Ancillary threads (e.g. windows) and vBrain
        functions may also request updates. 

        This function is implementation specific. Requires context locking.

        Parameters:
          groupId  - shadow group Id. (default: 'all')
        
        Return Value:
          Returns dictionary of current read sensor values.
    """
    pass


  # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  # vRobot (Cere)Bellum Member Functions
  # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

  #--
  def BellumInit(self):
    """ Initialize the vRobot's low-level bellum goals and data.

        The (cere)bellum integrates information from the sensory groups
        that indicates position, movement, etc. and uses this information
        to coordinate (reactive) movements.

        Return Value:
          None
    """
    self.mBellumGoalSet = {}      # create empty robot goal set
    self.BellumNullGoals()        # set goals to null set

  #--
  def BellumGetGoals(self):
    """ Get the current set of robot low-level goals.

        Return Value:
          Current goal set (implementation specific).
    """
    return self.mBellumGoalSet

  #--
  def BellumSetGoals(self, **kwGoals):
    """ Set new vRobot low-level goals. The old goal set is replaced by
        this new goal set.

        Parameters:
          kwGoals  - argument list of keyword=value goals(s) 
                     (implementation specific)
        
        Return Value:
          Returns the new current goal set. 
    """
    for k,v in kwGoals.iteritems():
      self.mBellumGoalSet[k] = v
    return self.mBellumGoalSet

  #--
  def BellumNullGoals(self):
    """ Set the vRobot's low-level goals to the null set
        (i.e. resting state).

        Return Value:
          Returns the new current goal set (implementation specific). 
    """
    return self.mBellumGoalSet


  # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  # vRobot State Member Function
  # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

  #--
  def SetCommStatus(self, commIsUp):
    """ Set the communication channel status.

        Parameters:
          commIsUp  - communication is [not] up

        Return Value:
          None
    """
    if commIsUp:
      self.mCommIsUp = True
      self.GSReportNormalStatus('Connection is up')
      if self.mServerState == Gluon.EServerState.NotReady:
        self.SetRobotState(Gluon.EServerState.Ready)
    else:
      self.mCommIsUp = False
      self.GSReportErrorStatus('Connection is down')
      if self.mServerState == Gluon.EServerState.Ready:
        self.SetRobotState(Gluon.EServerState.NotReady)
      if self.mServerState not in [Gluon.EServerState.None,
                                   Gluon.EServerState.NotLoaded,
                                   Gluon.EServerState.NotReady]:
        self.SetRobotState(Gluon.EServerState.NotReady)

  #--
  def IsCommUp(self):
    """ Return True/False depending on whether communication is up between
        vRobot and the real/simulated robot.

        Return Value:
          True if communication has been established and is operational,
          False otherwise.
    """
    return self.mCommIsUp

  #--
  def GetRobotState(self):
    """ Get the robot's current server state. Synonym for GSGetServerState()

        Return Value:
          Returns current state EServerState
    """
    return self.GSGetServerState()

  #--
  def SetRobotState(self, newState):
    """ Set the robot's new server state. Synonym for GSSetServerState()

        Parameters:
          newState  - new robot server state EServerState

        Return Value:
          Returns old robot server state.
    """
    return self.GSSetServerState(newState)


  # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  # vRobot Physical/Simulated Robot Member Functions
  # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

  #--
  def pRobotInit(self):
    """ Initialize the state of the physical/simulated robot.

        This function is implementation specific.

        Return Value:
          None.
    """
    pass

  #--
  def pRobotConnect(self, *args):
    """ Establish communicaton channel with the physical/simulated robot.

        This function is implementation specific.

        Parameters:
          args    - connection arguments

        Return Value:
          None.
    """
    pass

  #--
  def pRobotDisconnect(self):
    """ Disconnnect communicaton channel with the physical/simulated robot.
        The robot should be safed prior to closing the channel.

        This function is implementation specific.

        Return Value:
          None.
    """
    pass

  #--
  def pRobotHold(self):
    """ Hold the current position of the physical/simulated robot.

        This function is implementation specific.

        Return Value:
          None.
    """
    pass

  #--
  def pRobotRelease(self):
    """ Release the physical/simulated robot from any hold position.

        This function is implementation specific.
        
        Return Value:
          None.
    """
    pass
