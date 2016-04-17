#!python
###############################################################################
#
# RNProdStage.py
#

""" RoadNarrows Product Make Tools - Product Staging

The RNProdStage module creates a product staging area ready for making and
packaging.

Author: Robin D. Knight
Email:  robin.knight@roadnarrows.com
URL:    http://www.roadnarrows.com

Copyright (C) 2009.  RoadNarrows LLC.
All Rights Reserved

$LastChangedDate: 2010-08-04 15:01:38 -0600 (Wed, 04 Aug 2010) $
$Rev: 547 $
"""

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
import os
import shutil
import glob

import RNProdParam as Param
import RNProdXmlParser
import RNProdProdXml
import RNProdExceptXml
import RNProdReadMeXml
import RNProdAtAt

#-----------------------------------------------------------------------------
# Interactive Functions and Data
#-----------------------------------------------------------------------------

#--
def FSetPkgList(prodXml, fKey, val):
  while True:
    dirpath, sep, val = val.partition(',')
    dirpath = dirpath.strip()
    val = val.strip()
    if not sep and dirpath in ['end', 'done']:
      if len(prodXml[fKey]) > 0:
        return 'done'
      else:
        raise ValueError("at least one package is required")
    if not dirpath:
      raise ValueError("bad syntax")
    elif not os.path.isdir(dirpath):
      raise ValueError("%s: not a directory" % (repr(dirpath)))
    prodXml.AddElemPkg(dirpath)
    if not sep:
      return 'done'
    elif not val:
      return 'more'

#--
def FSetText(prodXml, fKey, val):
  if not val or val == '.':
    return 'done'
  s = prodXml[fKey] + '\n' + val
  while True:
    try:
      rsp = raw_input('> ')
    except KeyboardInterrupt:
      print "\n<INTR>"
      sys.exit(4)
    if not rsp:
      break
    else:
      s = s + '\n' + rsp
  prodXml.SetElemVal(fKey, s)
  return 'done'

# User Input Field Information
RNProdStageFInfo = {
    "stage_root":
    { "psname":   "Stage Root",
      "psfmt":    None,
      "set":      None
    },

    "rndiv":
    { "psname":   "Division",
      "psfmt":    'rn|rnr|rnis',
      "set":      None
    },

    "prod_name":
    { "psname":   "Product Name",
      "psfmt":    None,
      "set":      None
    },

    "prod_ver":
    { "psname":   "Product Version",
      "psfmt":    "maj.min.rev[-spec]",
      "set":      None
    },

    "synopsis":
    { "psname":   "Synopsis",
      "psfmt":    None,
      "set":      FSetText
    },

    "description":
    { "psname":   "Description",
      "psfmt":    None,
      "set":      FSetText
    },

    "rnmake":
    { "psname":   "RN Make Package Path",
      "psfmt":    None,
      "set":      None
    },

    "pkg_list":
    { "psname":   "Packages",
      "psfmt":    "path [, path ...][,]",
      "set":      FSetPkgList
    },

    "except_file":
    { "psname":   "Exceptions File",
      "psfmt":    None,
      "set":      None
    },

    "readme_file":
    { "psname":   "README File",
      "psfmt":    None,
      "set":      None
    },
}

#--
def YesOrNo(ps):
  while True:
    rsp = raw_input(ps+' {y|n} ')
    rsp = rsp.strip()
    if rsp in ['n', 'N', 'no', 'No', 'NO']:
      return 'no'
    elif rsp in ['y', 'Y', 'yes', 'Yes', 'YES']:
      return 'yes'
    else:
      print "huh?"

#--
def CollectUserData(prodXml):
  fOrder = prodXml.GetElemList()
  for fKey in fOrder:
    if not RNProdStageFInfo.has_key(fKey):    # skip 
      continue
    try:
      CollectUserField(prodXml, fKey)
    except KeyboardInterrupt:
      print "\n<INTR>"
      sys.exit(4)

#--
def CollectUserField(prodXml, fKey):
  fInfo = RNProdStageFInfo[fKey]
  ps = "RNProdStage: %s: " % (fInfo['psname'])
  if fInfo['psfmt']:
    ps += "(%s) " % (fInfo['psfmt'])
  if prodXml.HasPDataDft(fKey):
    ps += "{%s} " % (repr(prodXml.GetPDataDft(fKey)))
  hasVal = False
  while not hasVal:
    rsp = raw_input(ps)
    rsp = rsp.strip()
    if rsp == '?':
      print "%s:" % (RNProdStageFInfo[fKey]['psname']), repr(prodXml[fKey])
      continue
    elif rsp in ['exit', 'quit']:
      sys.exit(4)
    if not rsp:
      if prodXml.HasPDataDft(fKey):
        try:
          prodXml.SetElemVal(fKey, prodXml.GetPDataDft(fKey))
          hasVal = True
        except RNProdXmlParser.XmlParserError as inst:
          print "Error: %s: %s: %s." % (fInfo['psname'], repr(rsp), inst)
    elif fInfo['set']:
      try:
        rc = fInfo['set'](prodXml, fKey, rsp)
        if rc == 'done':
          hasVal = True
      except ValueError as inst:
        print "Error: %s: %s: %s." % (fInfo['psname'], repr(rsp), inst)
    else:
      try:
        prodXml.SetElemVal(fKey, rsp)
        hasVal = True
      except RNProdXmlParser.XmlParserError as inst:
        print "Error: %s: %s: %s." % (fInfo['psname'], repr(rsp), inst)
      

#-----------------------------------------------------------------------------
# Fileage
#-----------------------------------------------------------------------------

#--
def pjoin(p1, p2):
  return os.path.normpath(p1 + os.path.sep + p2)

#--
def FindNewest(pat):
  plist     = glob.glob(pat)
  newPath   = None
  newMtime  = 0.0
  for p in plist:
    mtime = os.path.getmtime(p)
    if mtime > newMtime:
      newPath   = p
      newMtime  = mtime
  return newPath

#--
def ProvMkDir(dirName):
  if not os.path.isdir(dirName):
    print "mkdir %s" % dirName
    os.mkdir(dirName, 0775)

#--
def SafeCpFile(src, dst, bReplace=True):
  if os.path.isfile(dst):
    if not bReplace:
      return dst
    bufile = dst + '.bak'
    print "renaming %s --> %s" % (dst, bufile)
    os.rename(dst, bufile)
  print "copying %s --> %s" % (src, dst)
  shutil.copyfile(src, dst)
  return dst

#--
def StripExtTemplate(p):
  root, ext = os.path.splitext(p)
  if ext == Param.RNProdTemplateExt:
    return root
  else:
    return p

#--
def SafeCpTemplate(template, dirDst, bReplace=True):
  fileDst = StripExtTemplate(template)
  pathTemplate  = pjoin(Param.RNProdFqDirTemplates, template)
  pathDst       = pjoin(dirDst, fileDst)
  SafeCpFile(pathTemplate, pathDst, bReplace)
  return pathDst


#-----------------------------------------------------------------------------
# Staging Functions
#-----------------------------------------------------------------------------

#--
def AtAtCbPkgInfoIter(fp, id, prodDict, fmt):
  _icnt = 0
  for _ivar in prodDict['pkg_info']:
    if _ivar['package'] == 'rnmake':
      continue
    s = eval(fmt)
    fp.write(s)
    _icnt += 1

#--
def AtAtCbPkgInfoAllIter(fp, id, prodDict, fmt):
  _icnt = 0
  for _ivar in prodDict['pkg_info']:
    s = eval(fmt)
    fp.write(s)
    _icnt += 1

#--
def AtAtCbPkgListIter(fp, id, prodDict, fmt):
  _icnt = 0
  for _ivar in prodDict['pkg_list']:
    s = eval(fmt)
    fp.write(s)
    _icnt += 1

#--
def AtAtCbRNDivEula(fp, id, prodDict, fmt):
  eulaFilename = pjoin(prodDict['stage_root'], prodDict['prod_fqname'])
  eulaFilename = pjoin(eulaFilename, StripExtTemplate(Param.RNProdTemplateEULA))
  fpIn = open(eulaFilename, 'r')
  if fmt == 'make':
    eulaStart   = "\n"
    linePrefix  = "# "
    eulaEnd     = ""
  elif fmt == 'c':
    eulaStart   = "\n/*\n"
    linePrefix  = " * "
    eulaEnd     = " */"
  elif fmt == 'c++':
    eulaStart   = "\n"
    linePrefix  = "// "
    eulaEnd     = ""
  elif fmt == 'html':
    eulaStart   = "\n<!--\n"
    linePrefix  = " - "
    eulaEnd     = " -->"
  else:
    eulaStart   = "\n"
    linePrefix  = ""
    eulaEnd     = ""
  eulaLines = fpIn.readlines()
  fpIn.close()
  fp.write(eulaStart)
  for line in eulaLines:
    fp.write(linePrefix)
    fp.write(line)
  fp.write(eulaEnd)

#--
def StageCleanup(fqdirStage):
  if not os.path.isdir(fqdirStage):
    return
  print "cleaning up stage"
  olddir = pjoin(fqdirStage, "dist")
  if os.path.isdir(olddir):
    shutil.rmtree(olddir)
  olddir = pjoin(fqdirStage, "xinstall")
  if os.path.isdir(olddir):
    shutil.rmtree(olddir)

#--
def StageSetProdXml(prodXml, dirDst):
  fileProdXml = pjoin(dirDst, 'Prod.xml')
  if os.path.isfile(fileProdXml):
    bufile = fileProdXml + '.bak'
    print "renaming %s --> %s" % (fileProdXml, bufile)
    os.rename(fileProdXml, bufile)
  print "creating %s" % (fileProdXml)
  prodXml.WriteXmlFile(fileProdXml)

#--
def StageGetPkg(fqdirStage, fqdirLog, srcdirPkg, pkgInfo):
  # locals
  pkgName   = os.path.basename(srcdirPkg)
  logName   = pjoin(fqdirLog, "%s-tarball-src.log" % (pkgName))
  dstdirPkg = pjoin(fqdirStage, pkgName)

  # clean up package
  print "cleaning %s source" % (pkgName)
  cmd ="make -C %s distclean >%s" % (srcdirPkg, logName)

  # tar up package source
  print "tarring %s source" % (pkgName)
  cmd ="make -C %s tarball-src >>%s" % (srcdirPkg, logName)
  ec = os.system(cmd)
  if ec != 0:
    print "Error: command exitcode=%d: %s" % (ec, repr(cmd))
    return False

  # move latest source tarball (the one just made)
  pat = pjoin(srcdirPkg, pjoin('dist', "%s-*-src.tar.gz" % (pkgName)))
  srcTarball = FindNewest(pat)
  if not srcTarball:
    print "Error: %s: cannot find latest source tarball" % (pkgName)
    return False
  dstTarball = pjoin(fqdirStage, "%s-src.tar.gz" % (pkgName))
  print "renaming %s --> %s" % (srcTarball, dstTarball)
  os.rename(srcTarball, dstTarball)

  # remove any old package
  if os.path.isdir(dstdirPkg):
    print "removing old package %s" % (dstdirPkg)
    shutil.rmtree(dstdirPkg)

  # untar source tarball
  cmd ="tar -C %s --preserve -xvzf %s >>%s" % (fqdirStage, dstTarball, logName)
  print "untarring %s" % (dstTarball)
  ec = os.system(cmd)
  if ec != 0:
    print "Error: command exitcode=%d: %s" % (ec, repr(cmd))
    return False

  # remove tarball
  print "removing tarball %s" % (dstTarball)
  os.remove(dstTarball)

  # move untarred pacakge to unversioned name
  pat = pjoin(fqdirStage, "%s-*" % (pkgName))
  dirUntarred = FindNewest(pat)
  if not dirUntarred or not os.path.isdir(dirUntarred):
    print "Error: %s: cannot find untarred source tarball" % (pkgName)
    return False
  print "renaming package %s --> %s" % (dirUntarred, dstdirPkg)
  os.rename(dirUntarred, dstdirPkg)

  # parse package readme file
  readme = pjoin(dstdirPkg, 'README.xml')
  print "parsing package %s" % (readme)
  readmeXml = RNProdReadMeXml.RNProdReadMeXml(readme)
  try:
    readmeXml.Parse()
  except RNProdXmlParser.XmlParserError as inst:
    print "Error: %s" % (inst)
    return False
  pkgInfo += [readmeXml.GetPDataDict()]

  return True

#--
def StageGetRNMake(prodXml, fqdirStage, fqdirLog, pkgInfo):
  # locals
  srcdirPkg = os.path.abspath(prodXml['rnmake'])
  StageGetPkg(fqdirStage, fqdirLog, srcdirPkg, pkgInfo)

#--
def StageGetExceptions(prodXml, dirDst):
  fileExcept = pjoin(dirDst, StripExtTemplate(Param.RNProdTemplateExcepts))
  if prodXml['except_file']:
    SafeCpFile(prodXml['except_file'], fileExcept, bReplace=False)
  else:
    SafeCpTemplate(Param.RNProdTemplateExcepts, dirDst, False)
  print "parsing %s" % (fileExcept)
  exceptXml = RNProdExceptXml.RNProdExceptXml(fileExcept)
  try:
    exceptXml.Parse()
  except RNProdXmlParser.XmlParserError as inst:
    print "Error: %s" % (inst)
    return None
  print "modifying %s" % (fileExcept)
  RNProdAtAt.AtAtReplace(fileExcept, prodXml.GetPDataDict())
  return exceptXml

#--
def StageGetEULAs(prodXml, dirDst):
  fileName = SafeCpTemplate(Param.RNProdTemplateEULA, dirDst)
  print "modifying %s" % (fileName)
  RNProdAtAt.AtAtReplace(fileName, prodXml.GetPDataDict())
  fileName = SafeCpTemplate(Param.RNProdTemplateEULAHtml, dirDst)
  print "modifying %s" % (fileName)
  RNProdAtAt.AtAtReplace(fileName, prodXml.GetPDataDict())

#--
def StageGetManifest(prodXml, dirDst):
  fileName = SafeCpTemplate(Param.RNProdTemplateManifest, dirDst)
  print "modifying %s" % (fileName)
  RNProdAtAt.AtAtReplace(fileName, prodXml.GetPDataDict())

#--
def StageGetSrcDocFiles(prodXml, dirDst):
  fileSrcDocIndex = pjoin(dirDst,
                          StripExtTemplate(Param.RNProdTemplateSrcDocIndex))
  srcDirDoxy  = pjoin(prodXml['rnmake'], 'doxy')
  srcDirImgs  = pjoin(srcDirDoxy, prodXml['rndiv']+'_images')
  srcFileCss  = prodXml['rndiv'] + '_doxy.css'
  dstDirImgs  = pjoin(dirDst, 'images')
  dstFileCss  = 'srcdoc_doxy.css'
  SafeCpTemplate(Param.RNProdTemplateSrcDocIndex, dirDst)
  SafeCpFile(pjoin(srcDirDoxy, srcFileCss), pjoin(dirDst, dstFileCss))
  if os.path.isdir(dstDirImgs):
    shutil.rmtree(dstDirImgs)
  print "copying %s --> %s" % (srcDirImgs, dstDirImgs)
  shutil.copytree(srcDirImgs, dstDirImgs)
  print "modifying %s" % (fileSrcDocIndex)
  RNProdAtAt.AtAtReplace(fileSrcDocIndex, prodXml.GetPDataDict())

#--
def MakeStage(prodXml):
  print "Creating RN Product Stage %s" % (prodXml['prod_fqname'])

  # local vars
  fqdirStage    = pjoin(prodXml['stage_root'], prodXml['prod_fqname'])
  fqdirDotBuild = pjoin(fqdirStage, Param.RNProdDirDotBuild)
  fqdirMake     = pjoin(fqdirStage, Param.RNProdDirMake)
  pkgInfo       = []

  # clean up any previoues staged files and directories
  StageCleanup(fqdirStage)

  # construct base directory tree
  ProvMkDir(fqdirStage)
  ProvMkDir(fqdirDotBuild)
  ProvMkDir(fqdirMake)

  # master product xml file
  StageSetProdXml(prodXml, fqdirDotBuild)

  # get packages
  for srcdirPkg in prodXml['pkg_list']:
    if not StageGetPkg(fqdirStage, fqdirDotBuild, srcdirPkg, pkgInfo):
      return 8

  # get rnmake 'special' package
  StageGetRNMake(prodXml, fqdirStage, fqdirDotBuild, pkgInfo)

  prodXml['pkg_info']           = pkgInfo
  prodXml['pkg_info_iter']      = AtAtCbPkgInfoIter
  prodXml['pkg_info_all_iter']  = AtAtCbPkgInfoAllIter
  prodXml['pkg_list_iter']      = AtAtCbPkgListIter

  # product exceptions xml file
  exceptXml = StageGetExceptions(prodXml, fqdirDotBuild)
  if not exceptXml:
    return 8
  for exceptList in ['prune_stage', 'prune_src', 'prune_srcdoc']:
    prodXml[exceptList]  = exceptXml[exceptList]

  # prune stage of unwanted directories and files
  for p in prodXml['prune_stage']:
    p = os.path.abspath(pjoin(fqdirStage, p))
    if not p.startswith(fqdirStage):
      print "Warning: %s: not a subdirectory/file of %s: ignoring" % \
          (repr(p), repr(fqdirStage))
    elif not os.path.isdir(p) and not os.path.isfile(p):
      print "Warning: %s: not a directory or a file" % (repr(p))
    else:
      print "pruning %s" % (p)
      shutil.rmtree(p)

  # top-level README.xml file
  if prodXml['readme_file']:
    fileName = SafeCpFile(prodXml['readme_file'],
      pjoin(fqdirStage, StripExtTemplate(Param.RNProdTemplateReadMe)))
  else:
    fileName = SafeCpTemplate(Param.RNProdTemplateReadMe, fqdirStage, False)
  print "modifying %s" % (fileName)
  RNProdAtAt.AtAtReplace(fileName, prodXml.GetPDataDict())

  # top-level EULA files
  StageGetEULAs(prodXml, fqdirStage)
  prodXml['rndiv_eula'] = AtAtCbRNDivEula

  # Manifest file
  StageGetManifest(prodXml, fqdirDotBuild)

  # product make file Prod.mk
  fileName = SafeCpTemplate(Param.RNProdTemplateProdMake, fqdirMake)
  print "modifying %s" % (fileName)
  RNProdAtAt.AtAtReplace(fileName, prodXml.GetPDataDict())

  # top-level Makefile
  fileName = SafeCpTemplate(Param.RNProdTemplateTLMake, fqdirStage)
  print "modifying %s" % (fileName)
  RNProdAtAt.AtAtReplace(fileName, prodXml.GetPDataDict())

  # source doc files
  StageGetSrcDocFiles(prodXml, fqdirMake)

#--
def PrintUserData(prodXml):
  print "\n"
  print "RN Product Staging Information"
  for fKey in prodXml.GetElemList():
    if not RNProdStageFInfo.has_key(fKey):    # skip 
      continue
    print "%-*s" % (20, RNProdStageFInfo[fKey]['psname']+":"),
    print prodXml[fKey]
  print
  

#-------------------------------------------------------------------------------
# Main
#-------------------------------------------------------------------------------

import sys
import getopt

_Argv0    = __file__
_CmdName  = 'RNProdStage'
_CmdVer   = '1.0.0'

#--
class Usage(Exception):
  """ Command-Line Options Usage Exception Class. """
  def __init__(self, msg):
    self.msg = msg

#--
def PrintUsageErr(emsg):
  """ Print Error Usage Message. """
  if emsg:
    print "%s: %s" % (_Argv0, emsg)
  else:
    print "%s: error" % (_Argv0)
  print "Try '%s --help' for more information." % (_Argv0)

#--
def PrintUsage():
  """ Print Command-Line Usage Message """
  print "usage: %s [options]..." % (_Argv0)
  print """Options and arguments:
  -x, --xml <file>           : Product XML input file

  -v, --version              : Display version strings and exit.
  -h, --help                 : Display this help and exit.
  """

#--
def GetOptions(argv=None, **kwargs):
  """ Staging Main """
  global _Argv0, _CmdName, _CmdVer

  if argv is None:
    argv = sys.argv

  _Argv0 = kwargs.get('argv0', __file__)

  try:
    try:
      opts, args = getopt.getopt(argv[1:], "?hvx:",
                                ['help', 'version', 'xml=', ''])
    except getopt.error, msg:
      raise Usage(msg)
    for opt, optarg in opts:
      if opt in ('-h', '--help', '-?'):
        PrintUsage()
        sys.exit(0)
      elif opt in ('-v', '--version'):
        print "%s %s" %(_Argv0, _CmdVer)
        sys.exit(0)
      elif opt in ('-x', '--xml'):
        kwargs['xml'] = optarg
  except Usage, err:
    PrintUsageErr(err.msg)
    sys.exit(2)

  return kwargs

#--
def ExecInteractive():
  prodXml = RNProdProdXml.RNProdProdXml()
  print "RN Product Staging Creation"
  print "  Enter '?' to get current value."
  print "  Lists are separated by ','s, terminated by no ',', 'end', or 'done'."
  CollectUserData(prodXml)
  PrintUserData(prodXml)
  if YesOrNo("Are the data correct?") == 'no':
    return  4 # RDK do something here like loop
  MakeStage(prodXml)
  return 0

#--
def ExecFile(prodXmlFile):
  prodXml = RNProdProdXml.RNProdProdXml(prodXmlFile)
  try:
    prodXml.Parse()
  except RNProdXmlParser.XmlParserError as inst:
    print "Error: %s" % (inst)
    return 8
  PrintUserData(prodXml)
  MakeStage(prodXml)
  return 0

#--
def main(argv=None, **kwargs):
  global _Argv0, _CmdName, _CmdVer
  
  kwargs = GetOptions(argv, **kwargs)
  if kwargs.has_key('xml') and kwargs['xml']:
    rc = ExecFile(kwargs['xml'])
  else:
    rc = ExecInteractive()
  return rc


#--
if __name__ == '__main__':
  sys.exit( main(argv=None) )
