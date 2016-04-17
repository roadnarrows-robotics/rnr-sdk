################################################################################
#
# Trip.py
#

""" 2D Trip Module

2D trip container class module.

Author: Robin D. Knight
Email:  robin.knight@roadnarrowsrobotics.com
URL:    http://www.roadnarrowsrobotics.com
Date:   2006.01.22

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

import math


#-------------------------------------------------------------------------------
# Global Data
#-------------------------------------------------------------------------------

# 3-tuple indices
X     = 0
Y     = 1
THETA = 2

# trip indices
FIRST   = 'first'   # first waypoint in the trip
PENULT  = 'penult'  # next to last waypoint in the trip
LAST    = 'last'    # last waypoint in the trip


#-------------------------------------------------------------------------------
# CLASS: trip
#-------------------------------------------------------------------------------
class trip:
  """ 2D Trip Container Class

      A 2D trip is defined by a set of waypoint triplets: (x, y, theta),
      where
        (x, y)  - planar location from the origin (0.0, 0.0)
        theta   - ccw orientation, in radians, of the object (robot) in
                  relation to a line through (x, y) parallel to the x-axis.

      The zero triplet (0.0, 0.0, 0.0) and x, y units have meaning only to
      the application controlling this trip class.

      Waypoints may be accessed through INDEX arguments. An INDEX is 
      either a symbolic name or an integer value. 
      A symoboic INDEX is one of:
        FIRST     -->   first waypoint in the trip
        PENULT    -->   next to last waypoint in the trip
        LAST      -->   last waypoint in the trip
        landmark  -->   known landmark name string
  """
  def __init__(self, *waypoints):
    """ Initialize trip instance.

        Parameters:
          waypoints - See the AddSubTrip() 'waypoints' parameter
    """
    self.Init(*waypoints)

  #--
  def _cvtwaypoint(self, *waypoint):
    """ Convert waypoint into 3-tuple (x, y, theta). """
    n = len(waypoint)
    if n == 3:
      return waypoint[X], waypoint[Y], waypoint[THETA]
    elif n == 1:
      return waypoint[0][X], waypoint[0][Y], waypoint[0][THETA]
    else:
      raise TypeError("trip waypoint '%s' is wrong size" % (repr(waypoint)))

  #--
  def _cvtindex(self, index):
    """ Convert [symbolic] index into real waypoint index. """
    if type(index) == str:
      if index == FIRST:
        index = 0
      elif index == LAST:
        index = self.mLast
      elif index == PENULT:
        index = self.mLast - 1
      else:
        try:
          index = self.mLandmarks[index]
        except KeyError:
          raise KeyError('landmark %s does not exist' % repr(index))
    if index < 0 or index > self.mLast:
      raise IndexError("waypoint index %s out of range" % repr(index))
    return index

  #--
  def Init(self, *waypoints):
    """ (Re)Initialize trip.

        Parameters:
          waypoints - See the AddSubTrip() 'waypoints' parameter
    """
    self.mWaypoints = []
    self.mLast      = -1
    self.mLandmarks = {}
    self.AddSubTrip(*waypoints)

  #--
  def HasNumWaypoints(self):
    return self.mLast + 1

  #--
  def AddWaypoint(self, *waypoint):
    """ Add a waypoint to the end of the trip.

        Parameters:
          waypoint - waypoint triplet. Format is one of:
                      x, y, theta
                      (x, y, theta)
        
        Return Value:
          Newly added waypoint triplet (x, y, theta)
    """
    triplet = self._cvtwaypoint(*waypoint)
    self.mWaypoints.append(triplet)
    self.mLast += 1

  #--
  def AddDWaypoint(self, dx, dy, dtheta):
    """ Add a delta waypoint to the end of the trip.
        The delta values are added to the last waypoint to create
        the new waypoint.

        Parameters:
          dx      - delta x
          dy      - delta y
          dtheta  - delta theta
        
        Return Value:
          Newly added waypoint triplet (x, y, theta)
    """
    last = self.mWaypoints[self.mLast]
    self.mWaypoints.append((last[X]+dx, last[Y]+dy, last[THETA]+dtheta))
    self.mLast += 1

  #--
  def AddDDistWaypoint(self, dx, dy, theta):
    """ Add a delta distance waypoint to the end of the trip.
        The delta values are added to the last waypoint to create
        the new waypoint.

        Parameters:
          dx      - delta x
          dy      - delta y
          theta   - theta is the new object orientation
        
        Return Value:
          Newly added waypoint triplet (x, y, theta)
    """
    last = self.mWaypoints[self.mLast]
    self.mWaypoints.append((last[X]+dx, last[Y]+dy, theta))
    self.mLast += 1

  #--
  def DelWaypoint(self, index):
    """ Delete the waypoint at the given index. Any effected landmarks
        will also be deleted.

        Parameters:
          index   - INDEX
        
        Return Value:
          None.
    """
    index = self._cvtindex(index)
    del self.mWaypoints[start:start+1]
    self.mLast -= 1
    for lname, lindex in self.mLandmarks.iteritems():
      if lindex == index:
        del self.mLandmarks[lname]

  #--
  def GetWaypoint(self, index):
    """ Get the waypoint at the given index.

        Parameters:
          index   - INDEX
        
        Return Value:
          Waypoint triplet (x, y, theta)
    """
    index = self._cvtindex(index)
    return self.mWaypoints[index]

  #--
  def AddSubTrip(self, *waypoints):
    """ Add a subtrip to end of the current trip.

        Parameters:
          waypoints - initial waypoint triplets. Format is one of:
                        trip instance
                        x, y, theta
                        (x, y, theta)
                        [(x, y, theta), ...]

        Return Value:
          None.
    """
    if len(waypoints) > 0:
      arg = waypoints[0]
      if isinstance(arg, trip):
        arg = arg.GetTrip()
      if type(arg) == list:
        for waypoint in arg:
          self.AddWaypoint(waypoint)
      elif waypoints:
        self.AddWaypoint(*waypoints)

  #--
  def DelSubTrip(self, start, end):
    """ Delete the subtrip starting and ending at the given waypoints.
        Any effected landmarks will also be deleted.

        Parameters:
          start   - INDEX of the start of the subtrip
          end     - INDEX of the end of the subtrip
        
        Return Value:
          None.
    """
    start = self._cvtindex(start)
    end = self._cvtindex(end) + 1
    if start >= end:
      raise IndexError('start trip index %s > end index %s' \
          % (repr(start), repr(end-1)))
    del self.mWaypoints[start:end]
    self.mLast -= (end - start)
    for name, index in self.mLandmarks.iteritems():
      if index >= start and index < end:
        del self.mLandmarks[name]

  #--
  def GetSubTrip(self, start, end):
    """ Get the subtrip starting and ending at the given waypoints.

        Parameters:
          start   - INDEX of the start of the subtrip
          end     - INDEX of the end of the subtrip
        
        Return Value:
          List of subtrip triplets [(x, y, theta)...].
    """
    start = self._cvtindex(start)
    end = self._cvtindex(end) + 1
    if start >= end:
      raise IndexError('start trip index %s > end index %s' \
          % (repr(start), repr(end-1)))
    return self.mWaypoints[start:end]

  #--
  def GetTrip(self):
    """ Get the current trip.
        
        Return Value:
          Full list of waypoint triplets [(x, y, theta)...].
    """
    return self.mWaypoints

  #--
  def AddLandmark(self, name, index):
    """ Add a named landmark at the given trip waypoint.

        Parameters:
          name    - landmark name string
          index   - INDEX
        
        Return Value:
          None
    """
    index = self._cvtindex(index)
    self.mLandmarks[name] = index

  #--
  def DelLandmark(self, name):
    """ Delete the named landmark.

        Parameters:
          name    - landmark name string
        
        Return Value:
          None
    """
    try:
      del self.mLandmarks[name]
    except KeyError:
      raise KeyError('landmark %s does not exist' % repr(name))

  def iterlegs(self, start=FIRST, end=LAST):
    """ Iteratate over [sub]trip legs where a trip leg is defined
        by adjacent pairs of waypoints.

        Parameters:
          start   - INDEX of the start of the subtrip
          end     - INDEX of the end of the subtrip
        
        Return Value:
          tripLegIter instance
    """
    return tripLegIter(self, start, end)

  def __eq__(self, trip):
    """ trip.__eq__(trip) <==> true iff all waypoints in both trips equal """
    if len(self.mWaypoints) != len(trip):
      return False
    i = 0
    for waypoint in trip:
      if waypoint != self.mWaypoints[i]:
        return False
      i += 1
    return True

  def __iadd__(self, waypoints):
    """ trip.__iadd__(waypoints)  <==>  trip.AddSubTrip(waypoints) """
    self.AddSubTrip(waypoints)
    return self

  def __iter__(self):
    """ trip.__iter__()  <==>  trip.mWaypoints.__iter__() """
    return self.mWaypoints.__iter__()

  def __len__(self):
    """ trip.__len__()  <==>  len(trip.mWaypoints) """
    return len(self.mWaypoints)

  def __ne__(self, trip):
    """ trip.__ne__(trip)  <==>  not trip.__eq__(trip) """
    return not self.__eq__(trip)

  def __repr__(self):
    """ trip.__repr__()  <==>  repr(trip.mWaypoints) """
    return repr(self.mWaypoints)

  def rot(self, phi):
    """ Rotate the trip ccw about the origin (0, 0) by phi radians.

        Parameters:
          phi   - ccw rotation in radians

        Return Value:
          None
    """
    rotWaypoints = []
    for p in self.mWaypoints:
      rotWaypoints.append((p[X] * math.cos(phi) - p[Y] * math.sin(phi),
                           p[X] * math.sin(phi) + p[Y] * math.cos(phi),
                           p[THETA] + phi))
    self.mWaypoints = rotWaypoints

  def trans(self, dx, dy):
    """ Translate the trip by the given deltas.

        Parameters:
          dx   - delta x from the 0.0
          dx   - delta y from the 0.0

        Return Value:
          None
    """
    trWaypoints = []
    for p in self.mWaypoints:
      trWaypoints.append((p[X] + dx, p[Y] + dy, p[THETA]))
    self.mWaypoints = trWaypoints


#-------------------------------------------------------------------------------
# CLASS: tripLegIter
#-------------------------------------------------------------------------------
class tripLegIter:
  """ Trip Leg Iterator Class. """
  def __init__(self, trip, start, end):
    """ initialize leg iterator. """
    self._trip = trip
    try:
      self._iterStart = self._trip._cvtindex(start)
    except IndexError:
      self._iterStart = self.mLast
    try:
      self._iterEnd   = self._trip._cvtindex(end)
    except IndexError:
      self._iterEnd = self.mLast

  def __iter__(self):
    """ leg iterator start. """
    return self

  def next(self):
    """ leg iterator next. """
    if self._iterStart >= self._iterEnd:
      raise StopIteration
    p1 = self._trip.mWaypoints[self._iterStart]
    self._iterStart += 1
    p2 = self._trip.mWaypoints[self._iterStart]
    return p1, p2


#-------------------------------------------------------------------------------
# Test Code
#-------------------------------------------------------------------------------

if __name__ == '__main__':
  def main():
    """ Trip Test Main """
    print 'assignment'
    t0 = trip()
    print t0
    t1 = trip([(0, 0, 0), (1, 1, .5), (2, .75, .3)])
    print t1
    t2 = trip(3, 4, math.pi)
    print t2
    t3 = trip(t1)
    print t2
    t3 += (44, 55, 0.0)
    print t3
    t3 += [(100, 200, -1), (101, 201, -2)]
    print t3
    t3 += t1
    print t3
    t3 += t0
    print t3

    print '\niterator'
    for p1 in t3:
      print p1

    print '\niterlegs()'
    for p1, p2 in t3.iterlegs():
      print p1, '-->', p2

    print '\niterlegs(3,5)'
    for p1, p2 in t3.iterlegs(3, 5):
      print p1, '-->', p2

  # run test
  main()

  print """
See also the unit test in GuiWinKheTrip.py. To test, run the python script
'GuiWinKheTrip.py' and choose the 'testpattern' option at the prompt."""

