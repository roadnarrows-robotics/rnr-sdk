################################################################################
#
# PyDebug.py
#

""" Debug Module

Python data and helper class to print debug information.

Note: This module will not be compiled into bytecode if optimization is 
      enabled (python -O ...)

TODO:
  - Is there a preprocessor equivalent? Makes if __debug__: x() one stmt
  - Fix dprintcall()

Author: Robin D. Knight
Email:  robin.knight@roadnarrowsrobotics.com
URL:    http://www.roadnarrowsrobotics.com
Date:   2004.08.25

Copyright (C) 2004, 2005, 2006.  RoadNarrows LLC.
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
# Public Interface
#-------------------------------------------------------------------------------

if __debug__:
  import sys
  import inspect
  import string

  #-----------------------------------------------------------------------------
  # Global Data
  #-----------------------------------------------------------------------------

  # Debugging levels
  DLOFF = 0                               # Debugging Off
  DL1, DL2, DL3, DL4, DL5 = 1, 2, 3, 4, 5 # Debugging Level

  #-----------------------------------------------------------------------------
  # CLASS: PyDebug
  #-----------------------------------------------------------------------------
  class PyDebug:
    """Debug Class.

    Python debug helper class. All debugging output will be directed to 
    fout (default: stdout).

    All instances of this class and subsequence calls to it should be 
    prefaced by "if __debug__:". In this way, optimized compiled bytecode 
    (python -O ...) will not include any debug code.

    Example:
      import PyDebug as dbg
      class foo:
        def __init__(self):
          if __debug__:
            self.debug = 
              dbg.PyDebug(preface=self.__class__.__name__, debuglevel=dbg.DL1)
          ...

      def bar():
        if __debug__: self.debug.d1print("bar", "none")
        ...
    """
    
    def __init__(self, preface = None, debuglevel = DLOFF, fout = None):
      """ Initialize PyDebug instance. 

          Parameters:
            preface     - preface string to all debug output
            debuglevel  - starting level of debugging
            fout        - debug open output file object (None == stdout)
      """
      # set preface string. default preface string is name of this class
      if not preface:
        self.mPreface = self.__class__.__name__
      else:
        self.mPreface = preface
      if len(self.mPreface) > 0:
        self.mPreface += ":"

      # set output file and debug level
      # Note: will set to off if level < DL1
      self.mFout = None
      self.On(debuglevel, fout)

    #--
    def __del__(self):
      self.Off()

    #--
    def On(self, debuglevel=DL1, fout=None):
      """ Turn on debugging at the given debug level. Default is DL1.

          All debugging at levels <= current level will execute.

          Parameters:
            debuglevel  - starting level of debugging
            fout        - debug open output file object (None == stdout)
      """
      # set output file
      self._setFout(fout)

      # set debug level
      self._setDebugLevel(debuglevel)

      self._dprint(self.mDebugLevel, "LEVEL %d DEBUGGING TURNED ON" % \
                    self.mDebugLevel)

    #--
    def Off(self):
      """ Turn off all debugging. """
      self._dprint(self.mDebugLevel, ("DEBUGGING TURNED OFF"))
      self.mDebugLevel = DLOFF

    #--
    def At(self, thislevel):
      """ Returns True/False if current debugging level includes
          this level.
      """
      if self.mDebugLevel != DLOFF and self.mDebugLevel >= thislevel:
        return True
      else:
        return False

    #--
    def d1print(self, *args):
      """ DL1 debug print. """
      self._dprint(DL1, *args)

    #--
    def d2print(self, *args):
      """ DL2 debug print. """
      self._dprint(DL2, *args)

    #--
    def d3print(self, *args):
      """ DL3 debug print. """
      self._dprint(DL3, *args)

    #--
    def d4print(self, *args):
      """ DL4 debug print. """
      self._dprint(DL4, *args)

    #--
    def d5print(self, *args):
      """ DL5 debug print. """
      self._dprint(DL5, *args)

    #--
    def dprintcall(self, debuglevel=DL4):
      """Debug prints calling function/method name plus arguments names and 
         values.

         RDK!!! doesn't seem to be a good way to do this - investigate more
      """
      if self.mDebugLevel != DLOFF and self.mDebugLevel >= debuglevel:
        # get caller's frame and arguments
        s=inspect.formatargvalues(
            *inspect.getargvalues(inspect.currentframe(1)) )
        #print "rdk!!!", repr(s)
        # strip of leading and trailing parens
        #s = s[1:-1]
        #print "rdk!!!", repr(s)
        self._dprint(debuglevel, s)

    #--
    def _dprint(self, debuglevel, *args):
      """ Do the real debug printing of the given formatted arguments. """
      if self.mDebugLevel != DLOFF and self.mDebugLevel >= debuglevel:
        print >>self.mFout, "DL" + repr(debuglevel) + ":", self.mPreface,
        for arg in args:
          print >>self.mFout, arg,
        print >>self.mFout
        self.mFout.flush()

    #--
    def _setFout(self, fout):
      """ Set output file. """
      if not fout:
        if not self.mFout:
          self.mFout = sys.stdout
      else:
        try:
          m = fout.mode
          if m == 'r':
            raise IOError
          self.mFout = fout
        except (NameError, AttributeError):
          print  self.__class__.__name__ + ":", fout, ": not a file object"
          self.mFout = sys.stdout
        except IOError:
          print  self.__class__.__name__ + ":", fout, ": read-only"
          self.mFout = sys.stdout
    
    #--
    def _setDebugLevel(self, debuglevel):
      """ Set debug level. """
      if debuglevel <= DLOFF:
        self.mDebugLevel = DLOFF
      elif debuglevel > DL5:
        self.mDebugLevel = DL5
      else:
        self.mDebugLevel = debuglevel
