################################################################################
#
# SM.py
#

""" State Machine Module

State Machine (sm) module supporting both pre and post transitions functions
plus a generator (decision) state output function.

Author: Robin D. Knight
Email:  robin.knight@roadnarrowsrobotics.com
URL:    http://www.roadnarrowsrobotics.com
Date:   2006.09.18

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

if __debug__: import Fusion.Utils.PyDebug as PyDebug


#-------------------------------------------------------------------------------
# CLASS: SM
#-------------------------------------------------------------------------------
class SM:
  """ State Machine Class """

  #--
  def __init__(self, smName, smTbl={}, s_init=None, dbgobj=None):
    """ Initialize state machine instance.

        Parameters:
          smName  - name of this state machine
          smTbl   - state machine table
                    {s_0: {opPre_0=<func>, opPost_0=<func>, opGen_0=<func>,
                             transList_m=[(i_01, s_01), ...]},
                      ...
                     s_n: {opPre_n=<func>, opPost_n=<func>, opGen_n=<func>,
                             transList_n=[(i_n1, s_n1), ...]}
                    }
                    with:
                      s_k                 - state k
                      opPre_k(s_cur, i)   - pre-transition function k.
                                            Default: None
                      opPost_k(s_next, i) - post-transition function k.
                                            Default: None
                      s_next=opGen_k(s_cur, i) - state generation function k
                                            that supercedes transList.
                                            Default: None
                      transList_k         - transition 2-tuple list k
                                            i_kj - input j for state k
                                                   '*' is all inputs
                                            s_kj - next state kj

          s_init  - initial starting state
          dbgobj  - PyDebug object. None will create the debug object at
                    level 3.
    """
    if smName:
      self.mSmName = smName
    else:
      self.mSmName = 'SM'

    # debugging
    if __debug__:
      if not dbgobj:
        self.mDbg = PyDebug.PyDebug(self.mSmName)
        self.mDbg.On(PyDebug.DL3)
      else:
        self.mDbg = dbgobj

    self.mSmTbl = smTbl
    self.s_cur  = s_init

    if __debug__: self.PrintTbl()
    if __debug__: self.mDbg.d3print("SM: state: %s" % repr(self.s_cur))

  #--
  def AddState(self, state, opPre=None, opPost=None, opGen=None, transList={}):
    """ Add new state entry to state table.

        Parameters:
          state     - new state
          opPre     - pre-transition function.
          opPost    - post-transition function
          opGen     - state generation function
          transList - transition 2-tuple list

        Return Value:
          None
    """
    print state, opPost.__name__, transList
    
  #--
  def Transition(self, input):
    """ Transition to next state, firing off any pre and post-transition
        functions, given the new input. If the current state has no
        state generation function and the input does not match any entry
        in the state transition list, then the next state is the current
        state (i.e. a loop transition).

        Parameters:
          input   - new input object. The object must be comparable
                    (i.e. '==')

        Return Value:
          None
    """
    if __debug__: self.mDbg.d3print("SM: input: %s" % repr(input))
    if self.mSmTbl[self.s_cur].has_key('opPre'):
      self.mSmTbl[self.s_cur]['opPre'](self.s_cur, input)
    if self.mSmTbl[self.s_cur].has_key('opGen'):
      s_next = self.mSmTbl[self.s_cur]['opGen'](self.s_cur, input)
    else:
      s_next = self.GetTrans(self.s_cur, input)
    self.s_cur = s_next
    if self.mSmTbl[self.s_cur].has_key('opPost'):
      self.mSmTbl[self.s_cur]['opPost'](self.s_cur, input)
    if __debug__: self.mDbg.d3print("SM: state: %s" % repr(self.s_cur))

  #--
  def GetTrans(self, state, input):
    """ Get the transition state from the transition list, given the 
        state, input pair.

        Parameters:
          state   - state to look up
          input   - input object. The object must be comparable (i.e. '==')

        Return Value:
          None
    """
    for i,s in self.mSmTbl[state]['transList']:
      if i == '*' or i == input:
        return s
    return state

  if __debug__:
    #--
    def PrintTbl(self):
      """ Debug print state table. """
      for state, params in self.mSmTbl.iteritems():
        self.PrintTblEntry(state, **params)

    #--
    def PrintTblEntry(self, state, opPre=None, opPost=None, opGen=None,
                            transList={}):
      """ Debug print state table entry. """
      if opPre: namePre = opPre.__name__
      else: namePre = 'None'
      if opPost: namePost = opPost.__name__
      else: namePost = 'None'
      if opGen: nameGen = opGen.__name__
      else: nameGen = 'None'
      self.mDbg.d4print('SM: state=%s: opPre=%s, opGen=%s, opPost=%s' % \
          (repr(state), namePre, nameGen, namePost))
      self.mDbg.d4print('SM:   transList=', repr(transList))
    


#-------------------------------------------------------------------------------
# Test Code
#-------------------------------------------------------------------------------

if __name__ == '__main__':

  #--
  def s0_post(state, input):
    print 's0_post(%s, %s)' % (repr(state), repr(input))

  #--
  def s1_post(state, input):
    print 's1_post(%s, %s)' % (repr(state), repr(input))

  #--
  def s2_post(state, input):
    print 's2_post(%s, %s)' % (repr(state), repr(input))

  #--
  def main():
    """ SM Unit Test Main """
    sm = SM('SMUnitTest',
      smTbl = {
        's0': {'opPost':s0_post, 'transList':[(1, 's1')]},
        's1': {'opPost':s1_post, 'transList':[(0, 's0'), (1, 's1'), (2, 's2')]},
        's2': {'opPost':s2_post, 'transList':[(1, 's1')]}
      },
      s_init='s0')

    print "Enter SM integer input value or 'quit' to quit"
    while True:
      i = raw_input("input value> ")
      if i == 'quit':
        break
      try:
        i = int(i)
      except ValueError:
        print "Enter SM integer input value or 'quit' to quit"
      sm.Transition(i)

  # run test
  main()
