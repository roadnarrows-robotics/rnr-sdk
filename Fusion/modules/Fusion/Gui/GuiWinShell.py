################################################################################
#
# GuiWinShell
#

""" Graphical User Interface Simple Shell Window

Graphical User Interface (GUI) Tkinter window provides I/O and
graphical support to simple shells. The shell instance runs in
a separate thread. The shell object must support the following
keyword arguements:
  fin=fin       - read from fin instead of the default stdin
  fout=fout     - write to fout instead of the default stdout
  closeonexit   - do [not] close serial port on exit.
Both fin and fout are mappped to the GUI terminal parent process.
Stderr remains unmapped for shell debugging. 

The GUI privdes style methods to the shell to control the font
styles of the input/output.

Note:
  A better implementation is a fork or fork-exec model. But two 
  major issues must be addressed: 1) python on Windows does not 
  support fork() and, much harder, 2) shared python objects between
  the child and parent processes must be implemented. For example,
  a robot cmd object might own a port resource that must be shared
  between the shell(child) and vRobot(parent) processes.
      
Author: Robin D. Knight
Email:  robin.knight@roadnarrowsrobotics.com
URL:    http://www.roadnarrowsrobotics.com
Date:   2005.12.19

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

import  os
import  sys
import  threading as thread
import  signal
import  time

try:
  import readline as rl  # optional module
except ImportError:
  pass

import  tkinter as tk
import  tkinter.font

import  Fusion.Utils.Tools as utils
import  Fusion.Gui.GuiWinText as GuiWinText
import  Fusion.Gui.GuiTypes as gt


#-------------------------------------------------------------------------------
# Global Data
#-------------------------------------------------------------------------------

#
# Special Characters
#
_Esc          = chr(27)
_CtrlC        = '\x03'    # interrupt
_CtrlD        = '\x04'    # eof (posix)
_CtrlH        = '\x08'    # erase
_CtrlM        = '\r'      # newline
_CtrlZ        = '\x1a'    # eof (windows)

#
# GuiTerm Escape Sequences
#
_EscSeqSOT       = '['                   # escape sequence start of text
_EscSeqEOT       = ']'                   # escape sequence end of text
_EscSeqPre       = _Esc + _EscSeqSOT     # escape sequence preamble
_EscSeqQuit      = _EscSeqPre+'quit'     # quit gui terminal and shell preamble
_EscSeqAddStyle  = _EscSeqPre+'addstyle' # add style preamble
_EscSeqDelStyle  = _EscSeqPre+'delstyle' # delete style preamble
_EscSeqSetStyle  = _EscSeqPre+'setstyle' # set current style preamble
_EscSeqDftStyle  = _EscSeqPre+'dftstyle' # set current style to default preamble

#
# Default Key Symbol map.
#   'keysym': (gui_out, ascii_code)
#
# Has this been done or is there a better way?
#
_KeySymDict = {
  'Return':     (None, '\n',),
  'Escape':     (None, _Esc),
  'BackSpace':  (None, _CtrlH),
  'Delete':     (None, _CtrlH),
  'Tab':        (None, '\t')
}

#
# Control Sequence Ctrl-<x> map
#   'keysym': (gui_out, ascii_code)
#
_KeySymDictCtrlSeq = {
  'c':   ('Ctrl-C\n', _CtrlC),
  'C':   ('Ctrl-C\n', _CtrlC),
  'd':   ('Ctrl-D\n', _CtrlD),
  'D':   ('Ctrl-D\n', _CtrlD),
  'h':   (None,       _CtrlH),
  'H':   (None,       _CtrlH),
  'm':   (None,       '\n'),
  'M':   (None,       '\n'),
  'z':   ('Ctrl-Z\n', _CtrlZ),
  'Z':   ('Ctrl-Z\n', _CtrlZ),
}

# EOF
_EOF  = utils.USEREOF

# Shell unique id counter
_ShCounter = 0

#
# Information on current shells 
#
_ShInfo = {}


#-------------------------------------------------------------------------------
# Shell -> GuiTerm Communication Functions
#-------------------------------------------------------------------------------

#--
def ThisShellHasAGui():
  """ Shell available GUI Terminal message funcion.

      Simple check if the calling shell has an attached GuiTerm.

      Return Value:
        Return True if the is a GUI, else False.
  """
  return thread.currentThread().getName() in _ShInfo

#--
def SendQuit(exitcode):
  """ Shell available GUI Terminal message funcion.

      Send quit message to terminal (and shell) to terminate.

      Parameters:
        exitcode  - shell exit code

      Return Value:
        None.
  """
  _SendEscSeq(_EscSeqQuit+_EscSeqEOT)

#--
def SendAddStyle(styleName, **kw):
  """ Shell available GUI Terminal message funcion.

      Add a font style to the GUI. Use SendSetStyle() to set the current
      style.

      Parameters:
        styleName - style name. If a style with this name already exist,
                  it will be replace by this new style.
        kwargs  - font style keyword arguments. Keys:
                    foreground (fg)
                      foreground color: '#rrggbb' in hexidecimal      
                    background (bg)
                      background color: '#rrggbb' in hexidecimal
                    offset
                      controls text offset from the baseline. use a positive
                      value for superscripts, a negative value for subscripts.
                    underline
                      if 1 then underline, else no underline

      Return Value:
        None.
  """
  if not ThisShellHasAGui():
    return
  tagkw = {}
  escSeq = _EscSeqAddStyle+','+styleName
  kwstr = ''
  for key, value in kw.items():
    if type(value) != str:
      value = repr(value)
    if key == 'foreground' or key == 'fg':
      kwstr += ','+'foreground'+'='+value
    elif key == 'background' or key == 'bg':
      kwstr += ','+'background'+'='+value
    elif key == 'offset':
      kwstr += ','+'offset'+'='+value
    elif key == 'underline' or key == 'ul':
      kwstr += ','+'underline'+'='+value
  if len(kwstr) > 0:
    escSeq += kwstr + _EscSeqEOT
    _SendEscSeq(escSeq)

#--
def SendDelStyle(styleName):
  """ Shell available GUI Terminal message funcion.

      Delete font style from the GUI's list of styles.

      Parameters:
        styleName - name of style

      Return Value:
        None.
  """
  if styleName:
    _SendEscSeq(_EscSeqDelStyle + ',' + styleName + _EscSeqEOT)

#--
def SendSetStyle(styleName):
  """ Shell available GUI Terminal message funcion.

      Set the GUI's current stdout font style.

      Parameters:
        styleName - name of style

      Return Value:
        None.
  """
  if styleName:
    _SendEscSeq(_EscSeqSetStyle + ',' + styleName + _EscSeqEOT)

#--
def SendSetDefaultStyle():
  """ Shell available GUI Terminal message funcion.

      Set the GUI's default stdout font style.

      Return Value:
        None.
  """
  _SendEscSeq(_EscSeqDftStyle + _EscSeqEOT)

#--
def _SendEscSeq(escSeq):
  """ Actually send the escape sequence.

      Parameters:
        escSeq - escape sequence string

      Return Value:
        None.
  """
  info = _ShInfo.get(thread.currentThread().getName())
  if not info:
    return
  info['fout'].write(escSeq)
  info['fout'].flush()


#-------------------------------------------------------------------------------
# GuiWinShell Function
#-------------------------------------------------------------------------------
def GuiWinShell(master, shell, cbDestroy=None, title='GUI Shell',
                        shargs=(), **kwargs):
  """ Create the shell in a child process, a Gui Terminal in the parent
      process, remapping standard I/O (sans stderr) between the two.

      Parameters:
        master      - GUI master of this window
        shell       - shell classobj of function
        cbDestroy   - callback to master gui when this window is destroyed
        title       - title of gui terminal window
        shargs      - shell arguments
        kwargs      - shell keyword and GuiWin arguments

      Return Value:
        Parent process: associated GuiTerm object
        Child process:  N/A 
  """
  global _ShCounter, _ShInfo

  # new shell id (and thread name)
  shId = 'shell%d' % _ShCounter
  _ShCounter += 1

  # new pipes to/from shell
  pipeShellIn  = os.pipe()
  pipeShellOut = os.pipe()

  _ShInfo[shId]         = {}
  _ShInfo[shId]['fin']  = os.fdopen(pipeShellIn[0], 'r', 0)
  _ShInfo[shId]['fout'] = os.fdopen(pipeShellOut[1], 'w', 0)

  # add to shell arguments
  kwargs['fin']         = _ShInfo[shId]['fin']
  kwargs['fout']        = _ShInfo[shId]['fout']
  kwargs['closeonexit'] = False
  kwargs['shell']       = shell

  # create shell thread
  _ShInfo[shId]['thread'] = thread.Thread(target=ShellThread, name=shId,
                                          args=shargs, kwargs=kwargs)

  # create gui terminal
  gterm = GuiTerm(master, shId, pipeShellIn[1], pipeShellOut[0],
                                title=title, **kwargs)

  # start the shell thread
  _ShInfo[shId]['thread'].start()

  return gterm

#--
def ShellSignalHandler(signal, eventmsg):
  """ Default SIGINT Shell Interrupt Handler """
  #print('SignalHandler', signal, file=sys.stderr)
  SendQuit(4)
  sys.exit(0)


#-------------------------------------------------------------------------------
# Funtion: ShellThread
#-------------------------------------------------------------------------------
def ShellThread(*args, **kwargs):
    # invoke the shell
    shell = kwargs['shell']
    sh = shell(*args, **kwargs)
    ec = sh.Run()
    try:
      SendQuit(ec)
    except:
      pass

#-------------------------------------------------------------------------------
# CLASS: GuiTerm
#-------------------------------------------------------------------------------
class GuiTerm(GuiWinText.GuiWinText):
  """ Gui Terminal Window Class """

  #--
  def __init__(self, master, shellId, fdGui2Shell, fdShell2Gui,
                    cbDestroy=None, title='GUI Shell', **options):
    """ Initialize the Terminal Window.

        Parameters:
          master      - GUI master of this window
          shellPid    - shell child process id
          fdGui2Shell - Gui write / Shell stdin file descriptor 
          fdShell2Gui - Shell stdout / Gui read file descriptor 
          cbDestroy   - callback to master when this window is destroyed
          title       - title of this window
          options     - GuiWin core options
    """
    self.mShellId     = shellId               # attached shell pid
    self.mFdGui2Shell = fdGui2Shell           # gui -> shell fd
    self.mFdShell2Gui = fdShell2Gui           # shell -> gui fd
    self.mCbDestroy   = cbDestroy             # master callback on destroy

    self.mCurTag      = 'default'             # current font style tag
    self.mDoQuit      = False                 # don't quit
    self.mKeySymMapper = self._keysymDefault  # input key symbol mapper
    self.mStdinBuf    = ''                    # stdin buffer
    self.mEscSeq      = None                  # escape sequence

    # terminal window
    GuiWinText.GuiWinText.__init__(self, master, 
                    title=title,
                    cbDestroy=cbDestroy,
                    fontTuple=gt.FontCour10Bold,
                    lineNums=False,
                    restState=tk.NORMAL,
                    wrap=tk.CHAR,
                    **options)

    # add default font style tag
    self.TagAdd('default', foreground=gt.ColorBlack)

    # text widget adjustments
    textWidget = self.GetTextWidget()
    textWidget.bind("<KeyPress>", self.CbStdinPress)    # bind stdin to keypress
    textWidget.bind("<KeyRelease>", self.CbStdinRelease) # and to key release

    # create and start stdout thread
    self.mStdoutThread = thread.Thread(target=self.StdoutThread,
          name='stdout')
    self.mStdoutThread.start()


  # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  # GuiWin Overrides
  # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

  #--
  def destroy(self):
    """ destroy window overide. Called if:
          - Shell sends a quit message
          - Ctrl-C (default action, shell may override)
          - Stdout thread exits
          - Gui Terminal exit ('x' or 'close' window bar action)
          - Parent window is destroyed
          - GuiTerm.destroy() called directly
    """
    if self.mDoQuit == True:    # already quitting
      return

    # disallow re-entry
    self.mDoQuit = True

    # kill shell thread if necessary
    info = _ShInfo[self.mShellId]
    th = info['thread']
    if th.isAlive():
      try:
        os.write(self.mFdGui2Shell, _EOF)
      except OSError:
        pass
      th.join(1.0)
    if th.isAlive():  # stubborn bugger
      try:
        os.write(self.mFdGui2Shell, _CtrlC)
      except OSError:
        pass
      th.join(1.0)
    if th.isAlive():
      print('%s did not die' % self.mShellId, file=sys.stderr)

    # kill stdout thread if necessary
    if thread.currentThread().getName() != 'stdout':
      th = self.mStdoutThread
      if th.isAlive():
        info['fout'].write(_EscSeqQuit+_EscSeqEOT)
        info['fout'].flush()
        th.join(1.0)
      if th.isAlive():
        print('StdoutThread did not die', file=sys.stderr)

    # close up I/O
    info['fin'].close()
    info['fout'].close()
    os.close(self.mFdGui2Shell)
    os.close(self.mFdShell2Gui)
    del _ShInfo[self.mShellId]

    # now destroy this window
    # make callback to window creator
    GuiWinText.GuiWinText.destroy(self)


  # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  # Term Window Specifics
  # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

  #--
  def StdoutThread(self, args=(), kwargs={}):
    """ Stdout Thread. Processes all output from shell's stdout.

        Note: Cannot destroy window in this thread under Window. 
              Must do it in the main thread, so start a timed callback.
    """
    #print('stdout thread: started', file=sys.stderr)
    while not self.mDoQuit:
      #
      # read shell output
      #
      try:
        shOut = os.read(self.mFdShell2Gui, 80)
      except (IOError, OSError) as msg:
        print('Stdoutthread: %s' % msg, file=sys.stderr)
        self.after(100, self.destroy)
        return

      #
      # process shell output
      #
      lineOut = ''
      for c in shOut:

        if self.mDoQuit:
          break

        # start of escape sequence
        elif c == _Esc:
          self.mEscSeq = c
          if lineOut:
            self.TextAdd(lineOut)
            lineOut = ''

        # collecting escape sequence
        elif self.mEscSeq:
          self.mEscSeq += c
          if self.mEscSeq[1] != _EscSeqSOT: # bad preamble, flush
            self.mEscSeq = None
          elif c == _EscSeqEOT:             # end of escape sequence
            if self._processEscSeq() == 'quit':
              self.after(100, self.destroy)
              return
            self.mEscSeq = None
          elif c == '\n':                   # bad sequence, flush
            self.mEscSeq = None

        # end of 'normal' output line
        elif c == '\n':
          lineOut += c
          self.TextAdd(lineOut)
          lineOut = ''

        # collecting output
        else:
          lineOut += c

      # flush to terminal remaining 'normal' characters
      if lineOut:
        self.TextAdd(lineOut)
        lineOut = ''

  #--
  def CbStdinPress(self, event):
    """ Stdin Key Press Callback. Processes key press to GUI display and 
        to shell's stdin.
    """
    #print('keypress', event.keycode, event.keysym, event.keysym_num,
    #        file=sys.stderr)
    textWidget = self.GetTextWidget()
    textWidget.mark_set(tk.INSERT, tk.END)  # move to end to echo input
    guistr, shstr = self.mKeySymMapper(event.keysym, event.keysym_num)
    #print('  maps to', repr(guistr), repr(shstr), file=sys.stderr)
    if guistr:
      self.TextAdd(guistr)
    if not shstr:
      pass
    elif shstr == _CtrlC or shstr == _EOF:
      os.write(self.mFdGui2Shell, shstr)
      self.mStdinBuf = ''
    elif shstr == _CtrlH:
      self.mStdinBuf = self.mStdinBuf[:-1]
    #elif shstr == _Esc:    # editing in vi
    #  self.mStdinBuf += shstr + 'k'
    #  os.write(self.mFdGui2Shell, self.mStdinBuf)
    #  self.mStdinBuf = ''
    elif shstr == '\n':
      self.mStdinBuf += shstr
      os.write(self.mFdGui2Shell, self.mStdinBuf)
      self.mStdinBuf = ''
      #rl.insert_text(shstr)
      #rl.redisplay()
      #s = rl.get_line_buffer()
      #print('rl.linebuf', repr(s), file=sys.stderr)
    else:
      self.mStdinBuf += shstr
      #rl.insert_text(shstr)

  #--
  def TextAdd(self, txt):
    """ TextAdd Wrapper. """
    if not self.mDoQuit and txt:
      GuiWinText.GuiWinText.TextAdd(self, txt, self.mCurTag)

  #--
  def CbStdinRelease(self, event):
    """ Stdin Key Release Callback. """
    #print('keyrelease', event.keycode, event.keysym, event.keysym_num,
              #file=sys.stderr)
    if event.keysym == 'Control_L' or event.keysym == 'Control_R':
      self.mKeySymMapper = self._keysymDefault

  #--
  def CbEditPaste(self):
    """ EditPaste callback override """
    textWidget = self.GetTextWidget()
    paste = textWidget.selection_get(selection='CLIPBOARD')
    GuiWinText.GuiWinText.CbEditPaste(self)
    start = 0
    while start < len(paste):
      idx = paste.find('\n', start)
      if idx >= 0:
        self.mStdinBuf += paste[start:idx+1]
        os.write(self.mFdGui2Shell, self.mStdinBuf)
        self.mStdinBuf = ''
      else:
        self.mStdinBuf += paste[start:]
        break
      start = idx + 1

  #--
  def _keysymDefault(self, keysym, num):
    """ Default key symbol mapper.

        Parameters:
          keysym    - symbolic key symbol string
          num       - numeric value associated wit keysym

        Return Value:
          The 2-tuple (guistr, shstr) where guistr is the string to 
          be displayed on GUI Terminal, and shstr is the string to
          send to the shell's stdin.
    """
    global _KeySymDict
    if keysym == 'Control_L' or keysym == 'Control_R':
      self.mKeySymMapper = self._keysymCtrlSeq
      return None, None
    elif num >= ord(' ') and num <= ord('~'):
      return None, chr(num)
    elif keysym in _KeySymDict:
      return _KeySymDict[keysym]
    else:
      return None, None

  #--
  def _keysymCtrlSeq(self, keysym, num):
    """ Control sequence key symbol mapper.

        Parameters:
          keysym    - symbolic key symbol string
          num       - numeric value associated wit keysym

        Return Value:
          The 2-tuple (guistr, shstr) where guistr is the string to 
          be displayed on GUI Terminal, and shstr is the string to
          send to the shell's stdin.
    """
    global _KeySymDictCtrlSeq
    if keysym in _KeySymDictCtrlSeq:
      return _KeySymDictCtrlSeq[keysym]
    elif num >= ord('a') and num <= ord('z'): # a to z
        num = num - ord('a') + 1
        return None, chr(num)
    elif num >= ord('A') and num <= ord('Z'): # A to Z
        num = num - ord('A') + 1
        return None, chr(num)
    else:
      return None, None

  #--
  def _processEscSeq(self):
    """ Process GUI Terminal directed escape sequence.
    
        Return Value:
          Return 'continue' for normal escape sequences. Return 'quit'
          for the quit escape sequence.
    """
    #print(repr(self.mEscSeq), file=sys.stderr)
    if not self.mEscSeq:
      return 'continue'
    self.mEscSeq = self.mEscSeq[:-1]    # strip ']'
    if self.mEscSeq.find(_EscSeqQuit) == 0:
      return 'quit'
    elif self.mEscSeq.find(_EscSeqAddStyle) == 0:
      argstr = self.mEscSeq.split(',')
      tag = argstr[1]
      kw = {}
      for keqv in argstr[2:]:
        arg = keqv.split('=')
        try:
          v = int(arg[1])
        except ValueError:
          v = arg[1]
        kw[arg[0]] = v
      self.TagAdd(tag, **kw)
    elif self.mEscSeq.find(_EscSeqDelStyle) == 0:
      argstr = self.mEscSeq.split(',')
      tag = argstr[1]
      self.TagDel(tag)
    elif self.mEscSeq.find(_EscSeqSetStyle) == 0:
      argstr = self.mEscSeq.split(',')
      tag = argstr[1]
      self.mCurTag = tag
    elif self.mEscSeq.find(_EscSeqDftStyle) == 0:
      self.mCurTag = 'default'
    else:
      print('unknown escape sequence:', repr(self.mEscSeq), file=sys.stderr)
    return 'continue'


#-------------------------------------------------------------------------------
# Unit Test Code
#-------------------------------------------------------------------------------

if __name__ == '__main__':
  class DemoShell:
    """ Demonstrate GUI Shell capabilities. """
    def __init__(self, args=(), **kwargs):
      """ Initialization """
      if 'fin' in kwargs:
        self.mFin = kwargs['fin']
      else:
        self.mFin = sys.stdin
      if 'fout' in kwargs:
        self.mFout = kwargs['fout']
      else:
        self.mFout = sys.stdout
  
    def Run(self):
      """ Run Loop for User Interactive Shell """
      print("    Simple Test Command Shell", file=self.mFout)
      print("(type 'help' for list of commands)", file=self.mFout)
      print("", file=self.mFout)
      print('mainloop', file=sys.stderr)
      n = 0
      while True:
        ps1 = '%d$ ' % n
        try:
          cmd = utils.user_input(ps1, self.mFin, self.mFout)
        except KeyboardInterrupt:
          print('interrupt', file=sys.stderr)
          return 2
        except EOFError:
          print('eof', file=sys.stderr)
          return 0
        except OSError as err:
          print(err, file=sys.stderr)
          return 4
        print('cmd', repr(cmd), file=sys.stderr)
        args = cmd.split()
        if not args:
          pass
        elif args[0] == 'quit':
          return 0
        elif args[0] == 'add':
          kw = {}
          n = 2
          while n < len(args)-1:
            kw[args[n]] = args[n+1]
            n += 1
          SendAddStyle(args[1], **kw)
        elif args[0] == 'del':
          SendDelStyle(args[1])
        elif args[0] == 'dft':
          SendSetDefaultStyle()
        elif args[0] == 'set':
          SendSetStyle(args[1])
        elif args[0] == 'help':
          print("""Help:
    add <name> <attr> <val>... - add new style with fg and bg colors
    del <name>                 - delete style
    dft                        - set current style to default
    set <name>                 - set current style
    help                       - this output
    quit                       - quit shell""", file=self.mFout)
        else:
          print('echo', repr(cmd), file=self.mFout)
        n += 1
  
  #--
  def main(argv=None):
    """ GuiWinShell Unit Test Main """
    root = tk.Tk()
    sh = GuiWinShell(root, DemoShell)
    root.mainloop()

  # run unit test
  main()
