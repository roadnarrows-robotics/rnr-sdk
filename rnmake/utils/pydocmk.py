#!/usr/bin/env python3
# Package:  RN Makefile System Utility
# File:     pydocmk.py
# Desc:     Make python documentation for python [sub]package.
# Usage:    pydocmk.sh -d <docroot> setup
#
# /*! \file */
# /*! \cond RNMAKE_DOXY*/

import sys
import os
import time
#import glob
import pydoc
import getopt

#------------------------------------------------------------------------------
# Globals
#------------------------------------------------------------------------------

DocRoot   = None    # document install root directory path
PkgRoot   = None    # RN package root
PyDocInfo = None    # pydoc meta info from setup.py
PkgInfo   = None    # package info from setup.py

# HTML @variable@ substitution names to values dictionary 
HtmlVars = {
  'PKG_NAME':       '',
  'PKG_VER':        '',
  'BRIEF':          '',
  'LONG_DESC':      '',
  'AUTHOR':         '',
  'EMAIL':          '',
  'ORG_FQ':         '',
  'ORG_INITIALS':   '',
  'ORG_URL':        '',
  'MOD_URL_LIST':   [],
  'MOD_LIST_ITER':  None,
}


#------------------------------------------------------------------------------
# Template Variable '@variable@' AtAt Processing
#------------------------------------------------------------------------------

import shutil
import re

_reAtAt = re.compile('@([a-zA-Z_]\w*)@|@([a-zA-Z_]\w*):(.+)@')
_Now    = time.localtime()

AtAtBuiltIns = {
  'THIS_YEAR':    repr(_Now.tm_year),
  'THIS_DATE':    "%d.%02d.%02d" % (_Now.tm_year, _Now.tm_mon, _Now.tm_mday),
  'THIS_TIME':    "%02d:%02d:%02d" % (_Now.tm_hour, _Now.tm_min, _Now.tm_sec),
}

#--
def AtAtIdentifier(match):
  if match.lastindex == 1:
    return (match.group(1), None)
  if match.lastindex == 3:
    return (match.group(2), match.group(3))
  else:
    return (None, None)

#--
def AtAtWriteValAtomic(fp, val, fmt):
  if fmt:
    fp.write(fmt % (val))
  elif type(val) == str:
    fp.write(val)
  else:
    fp.write(repr(val))

#--
def AtAtWriteValList(fp, valList, fmt):
  for val in valList:
    AtAtWriteValAtomic(fp, val, fmt)
    if not fmt:
      fp.write(' ')

#--
def AtAtWriteVal(fp, id, valDict, fmt=None):
  val = valDict[id]
  if val is None:
    pass
  elif callable(val):
    val(fp, id, valDict, fmt)
  elif type(val) == list:
    AtAtWriteValList(fp, val, fmt)
  else:
    AtAtWriteValAtomic(fp, val, fmt)

#--
def AtAtReplace(fileNameIn, userDict, bReplace=True):
  fileNameTmp = fileNameIn + '.tmp'
  fpSrc = open(fileNameIn, 'r')
  fpTmp = open(fileNameTmp, 'w')
  lineNum = 1
  line = fpSrc.readline()
  while line:
    #print("[%d] %s" % (lineNum, line), end='')
    m = 0
    for match in _reAtAt.finditer(line):
      #print(match.group(0), match.start(), match.end())
      fpTmp.write(line[m:match.start()])
      id,fmt = AtAtIdentifier(match)
      if id in userDict:
        AtAtWriteVal(fpTmp, id, userDict, fmt)
      elif id in AtAtBuiltIns:
        AtAtWriteVal(fpTmp, id, AtAtBuiltIns, fmt)
      else:
        AtAtWarning(fileNameIn, lineNum, match.start(), id, 
            "unknown identifier")
        fpTmp.write(match.group(0))
      m = match.end()
    if m < len(line):
      fpTmp.write(line[m:])
    lineNum += 1
    line = fpSrc.readline()
  fpSrc.close()
  fpTmp.close()
  if bReplace:
    os.rename(fileNameTmp, fileNameIn)
    return fileNameIn
  else:
    return fileNameTmp

#--
def AtAtWarning(fileName, lineNum, colNum, *args):
  """ Print AtAt warning.

      Parameters:
        *args   - List of warning message arguments.
  """
  wmsg = "Warning: %s[%d,%d]" % (fileName, lineNum, colNum)
  for a in args:
    wmsg += ": %s" %(a)
  print(wmsg)


#------------------------------------------------------------------------------
# Document Processing
#------------------------------------------------------------------------------

#--
def AtAtCbModUrlListIter(fp, id, varDict, fmt):
  _icnt = 0
  for _ivar in varDict['MOD_URL_LIST']:
    s = eval(fmt)
    fp.write(s)
    _icnt += 1

#--
def MakeEnv(docRoot, modSetup):
  global DocRoot, PkgRoot, PyDocInfo, PkgInfo
  dname = os.path.dirname(modSetup)
  if dname:
    sys.path = [dname] + sys.path
  else:
    sys.path = ['.'] + sys.path
  try:
    exec("import "+modSetup+" as setup")
  except ImportError:
    PrintUsageErr("%s: setup module not found - cannot import" % (modSetup))
    sys.exit(2)
  now = time.localtime()
  DocRoot   = docRoot
  PkgRoot   = setup.pkgroot
  PyDocInfo = setup.PyDocInfo
  PkgInfo   = setup.PkgInfo
  HtmlVars['PKG_NAME']    = PkgInfo['name']     # required
  HtmlVars['PKG_VER']     = PkgInfo['version']  # required
  HtmlVars['BRIEF']       = PkgInfo.get('description', '')
  HtmlVars['LONG_DESC']   = PkgInfo.get('long_description', '')
  HtmlVars['AUTHOR']      = PkgInfo.get('author', 'RoadNarrows')
  HtmlVars['EMAIL']       = PkgInfo.get('author_email','oneway@roadnarrows.com')
  HtmlVars['ORG_FQ']      = PkgInfo.get('maintainer', 'RoadNarrows LLC')
  HtmlVars['ORG_INITIALS'] = PyDocInfo.get('org_initials', 'RN')
  HtmlVars['ORG_URL']     = PkgInfo.get('url', 'http://www.roadnarrows.com/')
  HtmlVars['MOD_LIST_ITER'] = AtAtCbModUrlListIter
  for varname,imgfile in PyDocInfo['images'].iteritems():
    HtmlVars[varname] = imgfile

#--
def MakeDirs():
  if not os.path.exists(DocRoot):
    os.makedirs(DocRoot, mode=0o775)
  imgdir = DocRoot+os.sep+'images'
  if not os.path.exists(imgdir):
    os.mkdir(DocRoot+os.sep+'images', 0o775)

#--
def CopyImages():
  srcdir = PyDocInfo['images_dir']
  if not srcdir:
    return
  dstdir = DocRoot+os.sep+'images'+os.sep
  for varname,imgfile in PyDocInfo['images'].iteritems():
    shutil.copy(srcdir+os.sep+imgfile, dstdir)
  print("  Copied images.")

#--
def GetCbWalk(pyfilelist, dirname, fnames):
  excludes = ['.svn', '.deps' 'obj']
  for sEx in excludes:
    if dirname.find(sEx) != -1:
      return
  pyfilelist += [dirname]
  for f in fnames:
    if len(f) > 3 and f[-3:] == '.py':
      pyfilelist += [dirname+os.sep+f]

#--
def GetPythonFileList(modname):
    pyfilelist = []
    os.path.walk(modname, GetCbWalk, pyfilelist)
    return pyfilelist

#--
def MakeHtmlBatch(modbatch, htmlfilelist):
  if not modbatch:
    return 0
  cmd = "pydoc -w"
  for modpath in modbatch:
    cmd += ' '+modpath
  ec = os.system(cmd)
  if ec == 0:
    for modpath in modbatch:
      htmlfilelist += [modpath+'.html']
  return ec

#--
def MakeHtmlPkgDocs():
  #pydocumater = pydoc.HTMLDoc()  # not well documented
  for pkgdir in PkgInfo['package_dir'].itervalues():
    dname   = os.path.dirname(pkgdir)
    modname = os.path.basename(pkgdir)
    HtmlVars['MOD_URL_LIST'] += [modname + '.html']
    if dname:
      curdir = os.getcwd()
      os.chdir(dname)
    else:
      curdir = os.curdir
    #pydoc.writedocs('.', modname+'.') #os.curdir)  # strange behavior
    htmlfilelist = []
    pyfilelist = GetPythonFileList(modname)
    modbatch = []
    for pyfile in pyfilelist:
      bname=os.path.basename(pyfile)
      if bname[0] == '_':
        continue
      modpath = pyfile.replace(os.sep, '.')
      modpath = modpath.replace('.py', '')
      if len(modbatch) < 8:    # batch up to speed doc building
        modbatch += [modpath]
      else:
        MakeHtmlBatch(modbatch, htmlfilelist)
        modbatch = []
    if len(modbatch) > 0:
      MakeHtmlBatch(modbatch, htmlfilelist)
      modbatch = []
    os.chdir(curdir)
    for html in htmlfilelist:
      if dname:
        srcfile = dname + os.sep + html
      else:
        srcfile = html
      dstfile = DocRoot+os.sep+html
      try:
        os.rename(srcfile, DocRoot+os.sep+html)
      except OSError:
        print("%s -> %s: cannot copy" % (srcfile, dstfile), file=sys.stderr)

#--
def MakeHtmlIndex():
  postfile = AtAtReplace(PyDocInfo['index_template'], HtmlVars, False)
  os.rename(postfile, DocRoot+os.sep+'index.html')
  print('  Wrote index.html')

#--
def MakePyDoc(docRoot, modSetup):
  MakeEnv(docRoot, modSetup)
  MakeDirs()
  CopyImages()
  MakeHtmlPkgDocs()
  MakeHtmlIndex()


#------------------------------------------------------------------------------
# Main Execution
#------------------------------------------------------------------------------

_Argv0    = __file__

#--
class Usage(Exception):
  """ Command-Line Options Usage Exception Class. """
  def __init__(self, msg):
    self.msg = msg

#--
def PrintUsageErr(emsg):
  """ Print Error Usage Message. """
  if emsg:
    print("%s: %s" % (_Argv0, emsg))
  else:
    print("%s: error" % (_Argv0))
  print("Try '%s --help' for more information." % (_Argv0))

#--
def PrintUsage():
  """ Print Command-Line Usage Message """
  print(""""
usage: %s [OPTIONS] <setup_py_file>

     %s --help
  """  % (_Argv0, _Argv0))
  print("""Options and arguments:
-d, --docroot=<dir>      : Generated HTML documentation root directory
    --vpath=<path>       : Library virtual path

-h, --help               : Display this help and exit.
  """)

#--
def GetOptions(argv=None, **kwargs):
  """ Get Main Options and Arguments """
  global _Argv0

  if argv is None:
    argv = sys.argv

  _Argv0 = kwargs.get('argv0', __file__)

  # defaults
  kwargs['vpath'] = None
  kwargs['debug'] = 0

  # parse command-line options
  try:
    try:
      opts, args = getopt.getopt(argv[1:], "?hd:",
                        ['help', 'docroot=', 'vpath=', ''])
    except getopt.error as msg:
      raise Usage(msg)
    for opt, optarg in opts:
      if opt in ('-h', '--help', '-?'):
        PrintUsage()
        sys.exit(0)
      elif opt in ('-d', '--docroot'):
        kwargs['docroot'] = optarg
      elif opt in ('--vpath'):
        kwargs['vpath'] = optarg
  except Usage as err:
    PrintUsageErr(err.msg)
    sys.exit(2)

  if len(args) < 1:
    PrintUsageErr("No input setup.py file specified")
    sys.exit(2)
  else:
    kwargs['setup'] = args[0]

  return kwargs

#--
def Main(argv=None, **kwargs):
  """ Main """
  global _Argv0
  kwargs = GetOptions(argv, **kwargs)
  if kwargs['vpath'] is not None:
    ldpath = os.getenv('LD_LIBRARY_PATH')
    if ldpath:
      ldpath = kwargs['vpath'] + ':' + ldpath
    else:
      ldpath = kwargs['vpath']
    os.putenv('LD_LIBRARY_PATH', ldpath)
  MakePyDoc(kwargs['docroot'], kwargs['setup'])

#--
# Execute
#
sys.exit( Main() )

#/*! \endcond RNMAKE_DOXY */
