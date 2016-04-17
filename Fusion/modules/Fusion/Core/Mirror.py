################################################################################
#
# Mirror.py
#

""" Robotic Fusion Mirror Module.

The Mirror is the [a]synchronous request dispatcher for the Fusion Reactor.

Note: Synchronous calls do not work reliably - avoid.

Author: Robin D. Knight
Email:  robin.knight@roadnarrowsrobotics.com
URL:    http://www.roadnarrowsrobotics.com
Date:   2006.10.04

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

import sys
import Queue
import threading as thread
import time

import Fusion.Core.Values as Values
import Fusion.Core.Gluon as Gluon

import Fusion.Gui.GuiEvent as GuiEvent

if __debug__: import Fusion.Utils.PyDebug as PyDebug


#-------------------------------------------------------------------------------
# Global Data
#-------------------------------------------------------------------------------

#
# Builtin Mirror Requests
#
MirrorRequestRegister   = '##REGISTER##'    # register a request dispatcher 
MirrorRequestUnregister = '##UNREGISTER##'  # ungister a request dispatcher
MirrorRequestDie        = '##DIE##'         # kill mirror server thread


# Return Codes
Ok = 0


#-------------------------------------------------------------------------------
# CLASS: Mirror
#-------------------------------------------------------------------------------
class Mirror:
  """ Fusion Reactor Mirror Class.

      Mirror is a request dispatcher to carry out the actions of the
      Fusion client. The dispatcher runs in a separate thread. Fusion
      uses Mirror to off-load actions to protect the Tkinter mainloop()
      from blocking which causes all sorts of deadlocks.
  """

  def __init__(self, master, qsize=4, dbgobj=None):
    """ Initialize Mirror instance.

        Parameters:
          master      - Tkinter master widget (used for synchronization)
          qsize       - Request queue size (number of items)
          dbgobj      - PyDebug object. None will create the debug object at
                        level 2.
    """
    # debugging
    if __debug__:
      if not dbgobj:
        self.mDbg = PyDebug.PyDebug('Mirror')
        self.mDbg.On(PyDebug.DL2)
      else:
        self.mDbg = dbgobj

    # Event sychronization
    self.mSyncEvent = GuiEvent.GuiEvent(master)
    self.mSyncRsp   = None

    # Input Request Queue
    self.mQueue = Queue.Queue(qsize)

    # Dispatch Table
    self.mDispatchTbl = {}
    self.mDispatchTbl[MirrorRequestRegister]    = self._DispatchRegister
    self.mDispatchTbl[MirrorRequestUnregister]  = self._DispatchUnregister
    self.mDispatchTbl[MirrorRequestDie]         = self._DispatchThreadDie

    # create event dispatcher thread
    self._ThCreate()

  #--
  def __del__(self):
    """ Delete Mirror instance. """
    self._ThKill()

  #--
  def Quit(self):
    """ Quick way to kill Mirror bypassing the request queue. """
    self._ThKill()


  # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  # Mirror [A]Synchronous Request Member Functions
  # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

  def AsyncRequestRegister(self, request, callback):
    """ Asynchronous request to register a new request,callback pair
        for the dispatcher.

        Execution Context: Calling thread

        Parameters:
          request   - Request id to be registered.
          callback  - To be registered dispatch function associated with 
                      the new request id. The callback signature:
                        rsp = callback(request [,*args] [,**kwargs])
                      with rsp being any return value(s) from the callback.

        Return Value:
          None
    """
    return self._QueueRequest(MirrorRequestRegister, False, request, callback)

  def AsyncRequestUnregister(self, request):
    """ Asynchronous request to unregister an existing registered request 
        from the dispatcher.

        Execution Context: Calling thread

        Parameters:
          request   - request id to be unregistered.

        Return Value:
          None
    """
    self._QueueRequest(MirrorRequestUnregister, False, request)

  def AsyncRequestMirrorDie(self):
    """ Asynchronous request to kill the Mirror dispatcher thread.
        This request in effectively place at the head of the queue.

        Execution Context: Calling thread

        Return Value:
          None
    """
    self._ThKill()    # effectively move request to head of queue.
    self._QueueRequest(MirrorRequestDie, False)

  #--
  def AsyncRequest(self, request, *qargs, **qkwargs):
    """ Asynchronous request for Mirror thread to dispatch. The request must
        already be registered with Mirror.

        Execution Context: Calling thread

        Parameters:
          request   - registered request id
          **qargs   - queued arguments passed to registered dispatch.
          **qkwargs - queued keyword=value arguments passed to registered
                      dispatch.

        Return Value:
          None
    """
    self._QueueRequest(request, False, *qargs, **qkwargs);

  #--
  def SyncRequest(self, timeout, request, *qargs, **qkwargs):
    """ Synchronous request for the Mirror thread to dispatch. The
        caller will block until request is dispatched and a response 
        is returned. The request must already be registered with Mirror.

        Warning: Synchronous request do not work reliably. To synchonize,
                 install Tkinter after() callback at the end of dispatched
                 asynchronous Mirror callback().

        Execution Context: Calling thread

        Parameters:
          timeout   - Blocking timeout. A floating point value in seconds.
                      A timeout value of None or 0.0 will block forever
                      until the callback is dispatched and returns.
          request   - registered request id
          **qargs   - queued arguments passed to registered dispatch.
          **qkwargs - queued keyword=value arguments passed to registered
                      dispatch.

        Return Value:
          The response to the request where the response is simply the
          return value(s), if any, of the dispatched callback.
    """
    self.mSyncRsp = None
    self._QueueRequest(request, True, *qargs, **qkwargs)
    self.mSyncEvent.wait(timeout)
    return self.mSyncRsp

  #--
  def _QueueRequest(self, request, issync, *qargs, **qkwargs):
    """ Queue request for Mirror thread to dispatch. The Request must
        already be registered with Mirror.

        Execution Context: Calling thread

        Parameters:
          request   - registered request id
          issync    - is [not] a synchronous request-response transaction
          **qargs   - queued arguments passed to registered dispatch.
          **qkwargs - queued keyword=value arguments passed to registered
                      dispatch.

        Return Value:
          None
    """
    if not self.mDispatchTbl.has_key(request):
      err = 'Error: Request %s: not registered' % repr(request)
      if __debug__: self.mDbg.d3print('Mirror: %s' % err)
      raise ValueError(request)

    # try queueing with 0.1 second timeout
    try:
      self.mQueue.put((request, issync, qargs, qkwargs), True, 0.1)
      return Ok
    except Queue.Full:
      err = 'Error: Request %s: queue full' % repr(request)
      if __debug__: self.mDbg.d3print('Mirror: %s' % err)
      raise   # re-raise queue full


  # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  # Mirror Built-in Dispatch Functions
  # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

  def _DispatchRegister(self, request, request_to_reg, callback_to_reg):
    self.mDispatchTbl[request_to_reg] = callback_to_reg;

  def _DispatchUnregister(self, request, request_to_unreg):
    if self.mDispatchTbl.has_key(request_to_unreg):
      del self.mDispatchTbl[request_to_unreg]

  def _DispatchThreadDie(self, request):
    self._ThKill()


  # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  # Mirror Thread
  # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

  #--
  def _ThCreate(self):
    """ Create the dispatcher thread.

        Execution Context: Calling thread
    """
    self.mThCanLive   = True
    self.mThThread    = thread.Thread(target=self._ThThread,
                                      name='MirrorDispatcher', kwargs={})
    self.mThThread.start()

  #--
  def _ThThread(self, args=(), kwargs={}):
    """ The Mirror dispatcher thread. """
    thName = self.mThThread.getName()

    if __debug__: self.mDbg.d3print('%s: started' % thName)

    while self.mThCanLive:

      # block for request with 0.5 second timeout
      try:
        request, issync, qargs, qkwargs = self.mQueue.get(True, 0.5)
      except Queue.Empty:
        continue
 
      # dispatch
      if self.mDispatchTbl.has_key(request):
        if __debug__:
          self.mDbg.d3print('%s: dispatch, sync=%s: %s' % (thName, repr(issync),
            self._CallPrettyPrint(self.mDispatchTbl[request], request, *qargs,
              **qkwargs)))
        rsp = self.mDispatchTbl[request](request, *qargs, **qkwargs)
      else:
        if __debug__:
          self.mDbg.d3print('%s: unknown request: %s' % (thName, repr(request)))
        pass

      # synchronous request, unblock waiting thread with response
      if issync:
        if __debug__:
          self.mDbg.d3print('%s: sync response: %s' % (thName, repr(rsp)))
        self.mSyncRsp = rsp
        self.mSyncEvent.set()

    if __debug__: self.mDbg.d3print('%s: died' % thName)

  #--
  def _ThKill(self):
    """ Kill the dispather thread. """
    self.mThCanLive = False
 
  #--
  if __debug__:
    def _CallPrettyPrint(self, call, *args, **kwargs):
      """ Pretty print to string the dispatch function plus its arguments. """
      sep = ''
      s = call.__name__ + '('
      for a in args:
        s += sep+self._CallPrettyArg(a)
        sep = ','
      for k,v in kwargs.iteritems():
        s += sep+k+'='+self._CallPrettyArg(v)
        sep = ','
      s += ')'
      return s

    def _CallPrettyArg(self, arg):
      """ Pretty print to string a dispatch function argument. """
      if callable(arg):
        return arg.__name__+'()'
      elif isinstance(arg, Gluon.GluonServer):
        return arg.GSGetServerId()
      elif isinstance(arg, Gluon.GluonClient):
        return arg.GCGetClientId()
      else:
        return repr(arg)


#-------------------------------------------------------------------------------
# CLASS: Unit Test Code
#-------------------------------------------------------------------------------

if __name__ == '__main__':

  import Tkinter as tk
  import time

  # globals
  wlabel = None
  mirror = None
  cnt = 0

  #--
  def winsync():
    global mirror, wlabel
    rsp = mirror.SyncRequest(0, 'label')
    print 'winsync', repr(rsp)
    wlabel['text'] = rsp

  #--
  def winquit():
    global mirror, wlabel
    mirror.AsyncRequestMirrorDie()
    wlabel.master.destroy()

  #--
  def winbody(parent):
    global wlabel
    w = tk.Button(parent, text="Sync", command=winsync)
    w.grid(row=0, column=0)
    w = tk.Label(parent, text="synchronous text", width=80, borderwidth=1,
        justify=tk.LEFT, anchor=tk.W, background="#CCCC00")
    w.grid(row=0, column=1)
    wlabel = w
    w = tk.Button(parent, text="Quit", command=winquit)
    w.grid(row=1, column=0)

  #--
  def cbwinlabel(request):
    global cnt
    cnt += 1
    text = "synchronous update %d of label widget" % (cnt)
    print 'label:', repr(text)
    time.sleep(1.0)
    return text

  #--
  def cbecho(request, msg):
    print repr(msg)

  #--
  def cbshowcall(request, *args, **kwargs):
    print 'cbshowcall(%s, %s, %s)' % (repr(request), repr(args), repr(kwargs))

  #--
  def main():
    """ Mirror Unit Test Main """
    global mirror
    dbg = PyDebug.PyDebug('MirrorUT', PyDebug.DL3)
    root = tk.Tk()
    winbody(root)
    mirror = Mirror(root, dbgobj=dbg)
    rqList = [
      ['echo', '<msg>'],
      ['showcall', '[<arg>...] [key=val...]'],
      ['label', ''],
    ]
    mirror.AsyncRequestRegister(rqList[0][0], cbecho)
    mirror.AsyncRequestRegister(rqList[1][0], cbshowcall)
    mirror.AsyncRequestRegister(rqList[2][0], cbwinlabel)
    print "Enter <request> or 'help' for list of request"
    while True:
      request = raw_input("request> ")
      if not request:
        continue
      elif request == 'help':
        for rq in rqList:
          print rq[0], rq[1]
        print 'help'
        print 'quit'
      elif request == 'quit':
        mirror.AsyncRequestMirrorDie()
        root.destroy()
        break
      else:
        opts = request.split()
        request = opts[0]
        args = ()
        kwargs = {}
        for opt in opts[1:]:
          if opt.count('=') > 0:
            kv = opt.split('=')
            k = kv[0].strip()
            v = kv[1].strip()
            kwargs[k] = v
          else:
            args = args + (opt,)
        mirror.AsyncRequest(request, *args, **kwargs)

  # run unit test
  try:
    main()
  except SystemExit, status:
    sys.exit(status)
