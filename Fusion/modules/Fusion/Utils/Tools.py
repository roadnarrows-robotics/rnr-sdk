################################################################################
#
# Tools.py
#

""" Utility Tools Module.

The utilities' utilities.

Author: Robin D. Knight
Email:  robin.knight@roadnarrowsrobotics.com
URL:    http://www.roadnarrowsrobotics.com
Date:   2005.12.12

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

#-------------------------------------------------------------------------------
# Global Data
#-------------------------------------------------------------------------------

_CtrlC = '\x03'   # usual interrupt key
_CtrlD = '\x04'   # eof (posix)
_CtrlZ = '\x1a'   # eof (windows)

USERINT   = _CtrlC

if os.name == 'posix':
  USEREOF = _CtrlD
else:
  USEREOF = _CtrlZ


#-------------------------------------------------------------------------------
# Funtions
#-------------------------------------------------------------------------------

#--
def tuples2dict(tuplelist):
  """ Convert list of 2-tuples to dictionary equivalent.

      Parameters:
        tuplelist - list of 2-tuples [(val1, val2), ...]

      Return Value:
        Dictionary of {val1:val2, ...}
  """
  d = {}
  for name,value in tuplelist:
    d[name] = value
  return d

#--
def dict2tuples(d):
  """ Convert dictionary into equivalent list of 2-tuples.

      Parameters:
        d - Dictionary of {val1:val2, ...}

      Return Value:
        List of 2-tuples [(val1, val2), ...]
  """
  t = []
  for name,value in d.items():
    t += [(name, value)]
  return t

def newmodule(modname, filename=None):
  """ Create new, empty python module. 

      Original Code From:
        Steven D. Majewski (sdm7g@elvis.med.virginia.edu)
        Tue, 22 Mar 1994 18:41:17 -0500
      
      Parameters:
        modname   - unique (sys.modules) module name.
        filename  - optional [fully-qualified] python __file__ name.

      Return Value:
        Empty module reference.
        
  """
  _nullmodule = 'Fusion.Utils._nullmodule'
  import Fusion.Utils._nullmodule
  sys.modules[modname] = sys.modules[_nullmodule]
  sys.modules[modname].__name__ = modname
  if filename:
    sys.modules[modname].__file__ = filename
  del sys.modules[_nullmodule]
  del Fusion.Utils._nullmodule
  return sys.modules[modname]
                                                                                
def delmodule(module):
  """ Delete module from sys.modules and as a reference.

      Note: module reference is not deleted.

      Parameters:
        module    - imported module reference

      Return Value:
        None
  """
  modname = module.__name__
  if modname in sys.modules:
    del sys.modules[modname]
  #del module RDK!!! need a way to do this for real
                                                                                
def importmodule(filename, modname):
  """ Import python file as modname. 

      Original Code From:
        Steven D. Majewski (sdm7g@elvis.med.virginia.edu)
        Tue, 22 Mar 1994 18:41:17 -0500
      
      Exceptions Raised:
        ImportError   - cannot import file

      Parameters:
        filename  - OS specific [fully-qualified] python filename.
        modname   - unique (sys.modules) module name.

      Return Value:
        Imported module reference.
        
  """
  module = newmodule(modname, filename)
  dname = os.path.dirname(filename)
  cwd = os.getcwd()           # cd to local dir to ease import's
  sys.path.insert(0, dname)   # add local dir for filename 'import's
  try:
    os.chdir(dname)
  except:
    pass
  try:
    exec(compile(open(filename, "rb").read(), filename, 'exec'), module.__dict__, module.__dict__)
  except IOError as err:
    del sys.modules[modname]
    sys.path = sys.path[1:]
    os.chdir(cwd)
    raise ImportError(err)
  sys.path = sys.path[1:]
  os.chdir(cwd)
  return module
                                                                                

#--
def importfile(pathname):
  """ Import python file. Since python doesn't easily support importing
      an arbitrary python file [path]/fname.py, this function accomplishes
      the import by temporarily adding to the sys.path.

      The function is OBSOLETE. Use importmodule.

      Exceptions Raised:
        ImportError   - cannot import file

      Parameters:
        pathname  - OS specific path to the python file. If the pathname
                    ends in an '.py' extension, it will be automatically
                    stripped off to allow importing.

      Return Value:
        Imported module reference.
        
  """
  dname = os.path.dirname(pathname)
  fname = os.path.basename(pathname)
  if fname[-3:] == '.py':
    fname = fname[:-3]
  sys.path.insert(0, dname)
  try:
    mod = __import__(fname)
  finally:
    sys.path = sys.path[1:]
  return mod

#--
def canonicalpath(pathname):
  """ Canonicalize pathname.

      Parameter:
        pathname  - OS specific pathname string

      Return Value:
        Canonical pathname string on success, empty string '' on failure.
  """
  if not pathname:
    return ''
  pathname = os.path.expandvars(pathname) # expand all $xyz vars
  pathname = os.path.expanduser(pathname) # expand '~', etc
  if pathname.count('$') == 0:            # expansion worked
    return os.path.normpath(pathname)     # strip '//', etc
  else:
    return ''

#--
def canonicalexpr(expr):
  """ Canonicalize expresssion

      Parameter:
        expr  - OS specific expr string

      Return Value:
        Canonical expression string on success, empty string '' on failure.
  """
  if not expr:
    return ''
  expr = os.path.expandvars(expr) # expand all $<vars>. this works for any expr
  return expr 

#--
def canonicalsplitpaths(paths):
  """ Canonicalize pathnames in paths string.

      Parameter:
        paths  - os.pathsep separated paths string

      Return Value:
        List of canonical pathname strings.
  """
  if not paths:
    return []
  pathList = paths.split(os.pathsep)
  canonList = []
  for pathname in pathList:
    pathname = canonicalpath(pathname)
    if pathname:
      canonList += [pathname]
  return canonList

#--
def getpkgdir(pkg):
  """ Get the python package directory.

      Parameter:
        pkg  - python package name

      Return Value:
        Directory name if found, else None.
  """
  try:
    mod = __import__(pkg)
  except ImportError:
    return None
  try:
    pkgdir = mod.__path__[0]
    return pkgdir
  except (AttributeError, NameError, IndexError):
    pass
  try:
    pkgdir = mod.__file__
    return os.path.dirname(pkgdir)
  except (AttributeError, NameError, IndexError):
    return None

#--
def makefilename(*segArgs):
  """ Make OS-specific filename out of list of segments.

      Parameter:
        segArgs - directory and file name segment arguments.
                  Each segment may contain $<var> and '~' parts.

      Return Value:
        Canonicalized filename
  """
  filename = os.path.join(*segArgs)
  return canonicalpath(filename)

#--
def reopen(f, mode=None, bufsize=0):
  """ Reopen file f with the given mode and bufsize.

      Parameters:
        f       - file object
        mode    - new file mode. Default is current f's mode
        bufsize - buffering size. 
                    0   - unbuffered
                    1   - line buferred
                    >1  - buffer size (bytes)

      Return Value:
        New reopened file object
  """
  if mode is None: mode = f.mode
  fd = f.fileno()
  return os.fdopen(fd, mode, bufsize)

#--
def user_input(ps='', fin=sys.stdin, fout=sys.stdout):
  """ Prompt the user for input if the prompt string is given. Then
      read a line of user input. Any trailing newline is stripped. If
      the user hits EOF (Posix: Ctrl-D, Windows: Ctrl-Z), raise EOFError.
      KeyboardInterrupt may also be raised.

      Note: If input and output files are connected to a terminal, then
            readline features are available. RDK: I need to do this for
            pipes also, but unclear how to do this yet.

      Parameters:
        ps    - prompt string
        fin   - opened input file stream.
        fout  - opened output file stream.

      Return Value:
        User input string sans newline.
  """
  if os.isatty(fin.fileno()) and os.isatty(fout.fileno()):
    return input(ps)  # get line editing
  if ps:
    fout.write(ps)
    fout.flush()
  line = ''
  while True:
    c = fin.read(1)
    if c == USEREOF:
      raise EOFError
    elif c == USERINT:
      raise KeyboardInterrupt
    elif c == '\n':
      return line
    else:
      line += c
