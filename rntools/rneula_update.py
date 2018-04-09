#!/usr/bin/python

# //////////////////////////////////////////////////////////////////////////////
#
# Package:  tools
#
# File:     rneula_update.py
#
# Usage: rneula_update.py [OPTIONS] <license> [<dir> | <file>]
#
# Description:
#   RN package utility.
#
#   Update package source file EULA's.
#
# Author: Robin Knight (robin.knight@roadnarrows.com)
#
# Copyright (C) 2013-2018.  RoadNarrows LLC.
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
# //////////////////////////////////////////////////////////////////////////////

import os
import sys
import shutil
import fnmatch
import re
import getopt

RNMakeEnv = { }

def initRNMakeEnv():
  RNMakeEnv['rnmake'] = os.getenv('RNMAKE_ROOT', 'UNDEF')
  RNMakeEnv['arch_dft'] = os.getenv('RNMAKE_ARCH_DFT', 'x86_64')
  RNMakeEnv['xprefix'] = os.getenv('RNMAKE_INSTALL_XPREFIX')
  RNMakeEnv['prefix'] = os.getenv('RNMAKE_INSTALL_PREFIX')
  import __main__ as main
  RNMakeEnv['argv0'] = os.path.realpath(main.__file__)
  RNMakeEnv['tool_templates'] = os.path.dirname(RNMakeEnv['argv0']) \
      + '/templates'


## EULA templates root directory
EulaTemplatesRoot = None

EulaTagBegin    = "EulaBegin"
EulaTagEnd      = "EulaEnd"
reEulaTagBegin  = r"@EulaBegin@|\\EulaBegin|@EulaBegin"
reEulaTagEnd    = r"@EulaEnd@|\\EulaEnd|@EulaEnd"

## Excluded directories
Pat_excludes  = [
    '.svn', 'obj', '.deps', 'dist', 'loc', 'hw', 'gtest', 'tinyxml',
    'build', 'build.*', 'doxy', 'deb-doc', 'deb-dev', 'deb-src'
]

## Source file patterns
Pat_c_src     = ['*.[cCh]', '*.cxx', '*.cpp', '*.[ch][ch]', '*.hpp']
Pat_mk_src    = ['Makefile', 'makefile', '*.mk']
Pat_py_src    = ['*.py']
Pat_j_src     = ['*.java']
Pat_xml_src   = ['*.htm', '*.html', '*.xml']
Pat_doxy_src  = ['*.doxy']
Pat_js_src    = ['*.js']
Pat_src       = Pat_c_src + Pat_mk_src + Pat_py_src + Pat_j_src + \
                Pat_xml_src + Pat_doxy_src + Pat_js_src

## Doxygen file comment style patterns
Pat_slashstar_style = Pat_c_src + Pat_j_src + Pat_doxy_src + Pat_js_src + \
                      Pat_mk_src
Pat_hash_style      = Pat_py_src
Pat_hashhash_style  = []
Pat_hashbang_style  = []
Pat_xml_style       = Pat_xml_src
Pat_nocom_style     = []

## Slash-star (c, java, doxygen, php, ...) comment style constructs
ComSlashStarKey     = "/* */"
ComSlashStarStart   = "/*!"
ComSlashStarMiddle  = " * "
ComSlashStarEnd     = " */"

## Hash (makefiles, python, perl, ...) comment style constructs
ComHashKey          = "#"
ComHashStart        = "##"
ComHashMiddle       = "# "
ComHashEnd          = "#"

## Hash-hash doxygen comment style constructs
ComHashHashKey      = "##"
ComHashHashStart    = "##"
ComHashHashMiddle   = "##"
ComHashHashEnd      = "##"

## Hash-bang doxygen filtered pattern comment style constructs
ComHashBangKey      = "##!"
ComHashBangStart    = "##!"
ComHashBangMiddle   = "##!"
ComHashBangEnd      = "##!"

## XML (html, xml, ...) comment style constructs
ComXmlKey           = "<!-- -->"
ComXmlStart         = "<!--"
ComXmlMiddle        = " - "
ComXmlEnd           = " -->"

## No comment doxygen comment style constructs
NoComKey        = "nocomment"
NoCom           = ""

## Doxygen Supported Comments 
CommentStyle = {
    ComSlashStarKey: {
      'start':  ComSlashStarStart,
      'middle': ComSlashStarMiddle,
      'end':    ComSlashStarEnd
    },
    ComHashKey: {
      'start':  ComHashStart,
      'middle': ComHashMiddle,
      'end':    ComHashEnd
    },
    ComHashHashKey: {
      'start':  ComHashHashStart,
      'middle': ComHashHashMiddle,
      'end':    ComHashHashEnd
    },
    ComHashBangKey: {
      'start':  ComHashBangStart,
      'middle': ComHashBangMiddle,
      'end':    ComHashBangEnd
    },
    ComXmlKey: {
      'start':  ComXmlStart,
      'middle': ComXmlMiddle,
      'end':    ComXmlEnd
    },
    NoComKey: {
      'start':  NoCom,
      'middle': NoCom,
      'end':    NoCom
    }
}

## \brief Command-line exception class.
#
# Raise usage excpetion.
#
class usage(Exception):

  ## \brief Constructor.
  #
  # \param msg   Error message string.
  #
  def __init__(self, msg):
    ## error message attribute
    self.msg = msg


## \brief Application class.
#
class Application():

  ## \brief Constructor.
  #
  def __init__(self):
    ## command name
    self._Argv0 = __file__

    ## EULA license tag
    self.m_EulaLicenseTag = None

    ## EULA plain text filename
    self.m_EulaPlainTextFile = None

    ## Dictionary of EULAs
    self.m_EulaTag = { }
    self.m_Eula    = { }

    ## Number of files (required to be) updated.
    self.m_nUpdateCnt = 0

  ## \brief Print usage error.
  #
  # \param emsg  Error message string.
  #
  def printUsageErr(self, emsg):
    if emsg:
      print "{0}: {1}".format(self._Argv0, emsg)
    else:
      print "{0}: error".format(self._Argv0)
    print "Try '{0} --help' for more information.".format(self._Argv0)

  ## \brief Print Command-Line Usage Message.
  #
  def printUsage(self):
    print \
"""
Usage: {0} [OPTIONS] <license> [<dir>]
       {0} [OPTIONS] <license> [<file>]
       {0} --help

Update source file EULAs with the specified <license>.

Description:
  If a directory <dir> is specified, then recursively update all source files
  with the EULA of the specified by <license>. Only files that differ in
  <license> will be updated, unless the --force option is specified.
  Source files are defined as:
    C, C++, Java, Python, make files, doxygen, html, xml

  If a regular <file> is specified, then update the file's EULA with the
  specified <license>.

  If neither <dir> nor <file> is specified, the default is '.' - the current
  working directory.

Supported licenses can be found in {1}. See help for list.

Options and arguments:
-n, --noupdate            : List files that need updating, but don't update.
-f, --force               : Force update, even if EULAs match. Must be good.
-e, --exclude <dir>       : Exclude directory. May be iterated.


-h, --help                : Display this help and exit.
""".format(self._Argv0, EulaTemplatesRoot)
 
  ## \brief Print list of availabe license keys.
  #
  def printLicenseKeys(self):
    licenses = []
    for root, dirlist, filelist in os.walk(EulaTemplatesRoot):
      for fname in filelist:
        if fnmatch.fnmatch(fname, 'eula_*.txt'):
          licenses.append(fname[5:-4])
    print \
"""

The <license> is one of: {0}
""".format(licenses)

  ## \brief Print file i/o warning message.
  #
  # \param f        Opened file object or file name.
  # \param warnmsg  Warning message with {n} markup starting at 0.
  # \param args     Argument list to warnmsg.
  #
  def iowarning(self, f, warnmsg, *args):
    if type(f) is file:
      print "Warning: {0}:".format(f.name),
    else:
      print "Warning: {0}:".format(f),
    print warnmsg.format(*args)

  ## \brief Print file i/o error message.
  #
  # \param f        Opened file object or file name.
  # \param errmsg   Error message with {n} markup starting at 0.
  # \param args     Argument list to errmsg.
  #
  def ioerror(self, f, errmsg, *args):
    if type(f) is file:
      print "Error: {0}:".format(f.name),
    else:
      print "Error: {0}:".format(f),
    print errmsg.format(*args)


  ## \brief Get command-line options
  #  
  # \param argv          Argument list. If not None, then overrides
  #                      command-line arguments.
  # \param [out] kwargs  Keyword argument list.  
  #
  def getOptions(self, argv=None, **kwargs):
    if argv is None:
      argv = sys.argv

    self._Argv0 = kwargs.get('argv0', __file__)

    # defaults
    kwargs['debug']   = 0
    kwargs['update']  = True
    kwargs['force']   = False
    kwargs['topdir']  = '.'
    kwargs['file']    = None

    # parse command-line options
    try:
      try:
        opts, args = getopt.getopt(argv[1:], "?hnfe:",
            ['help', 'noupdate', 'force', 'exclude=', ''])
      except getopt.error, msg:
        raise usage(msg)
      for opt, optarg in opts:
        if opt in ('-n', '--noupdate'):
          kwargs['update'] = False
        elif opt in ('-f', '--force'):
          kwargs['force'] = True
        elif opt in ('-e', '--exlude'):
          Pat_excludes.append(optarg)
        elif opt in ('-h', '--help', '-?'):
          self.printUsage()
          self.printLicenseKeys()
          sys.exit(0)
    except usage, err:
      self.printUsageErr(err.msg)
      sys.exit(2)

    if len(args) < 1:
      self.printUsageErr("No EULA license specified.")
      sys.exit(2)
    else:
      kwargs['license'] = args[0]

    if len(args) > 1:
      if os.path.isdir(args[1]):
        kwargs['topdir'] = args[1]
      else:
        kwargs['file'] = args[1]

    return kwargs

  ## \brief Classify comment style from file name.
  #
  # \param fname   File name.
  #
  # \return Comment style.
  #
  def classifyFileType(self, fname):
    for pat in Pat_slashstar_style:
      if fnmatch.fnmatch(fname, pat):
        return ComSlashStarKey
    for pat in Pat_hash_style:
      if fnmatch.fnmatch(fname, pat):
        return ComHashKey
    for pat in Pat_hashhash_style:
      if fnmatch.fnmatch(fname, pat):
        return ComHashHashKey
    for pat in Pat_hashbang_style:
      if fnmatch.fnmatch(fname, pat):
        return ComHashBangKey
    for pat in Pat_xml_style:
      if fnmatch.fnmatch(fname, pat):
        return ComXmlKey
    for pat in Pat_nocom_style:
      if fnmatch.fnmatch(fname, pat):
        return NoComKey
    return None  

  ## \brief Add EULA of the given comment style to the dictionary of EULAs
  #
  # \param style    Comment style.
  #
  def addEulaStyle(self, style):
    comment = CommentStyle[style]
    self.m_Eula[style] = []
    for line in self.m_Eula['plaintext']:
      self.m_Eula[style].append(comment['middle']+line)
    self.m_EulaTag[style] = { }
    self.m_EulaTag[style]['begin'] = comment['middle']+"\\"+EulaTagBegin+"\n"
    self.m_EulaTag[style]['end']   = comment['middle']+"\\"+EulaTagEnd+"\n"
  
  ## \brief Find EULA block in open source file.
  #
  # \param f  Open source file object.
  #
  # \return
  # Returns a 4-tuple (found, nELineBegin, nELineEnd, eula) where:\n
  # value | description
  # ----- | -----------
  # found       | True or False if EULA block is found.
  # nELineBegin | The line number of the beginning of EULA block.
  # nELineEnd   | The line number of the end of EULA block.
  # eula        | The text holding the current file EULA found between the
  #               lines numbers.
  #
  def findEula(self, f):
    nELineBegin = -1      # begin tag line number
    nELineEnd   = -1      # end tag line number
    eula        = []      # eula block

    fail = (False, -1, -1, [])

    n = 0       # working line number

    for line in f:
      n += 1

      # begin tag
      if re.search(reEulaTagBegin, line):
        nELineBegin = n
      # end tag
      elif re.search(reEulaTagEnd, line):
        nELineEnd = n
        break
      # in eula block
      elif nELineBegin > 0:
        # eula too long
        if n-nELineBegin > 100:
          self.iowarning(f, "EULA block starting at line {0} is too long or " \
              "the '{1}' tag missing.", nELineBegin, EulaTagEnd)
          return fail
        # add line to eula block
        else:
          eula.append(line)

    #
    # final checks
    #

    # no eula in file
    if nELineEnd < 0 and nELineBegin < 0:
      self.iowarning(f, "No EULA found.", EulaTagBegin)
      return fail
    # no eula begin pattern
    elif nELineBegin < 0:
      self.iowarning(f, "'%{0} tag is missing or non-standard.", EulaTagBegin)
      return fail
    # no eula end pattern
    elif nELineEnd < 0:
      self.iowarning(f, "'%{0} tag is missing or non-standard.", EulaTagEnd)
      return fail
    # begin pattern found but end position preceeds it
    elif nELineEnd <= nELineBegin:
      self.iowarning(f,
          "EULA begin pattern at {0} follows end pattern at {1}.",
           nELineBegin, nELineEnd)
      return fail
    # got a good eula block
    else:
      return True, nELineBegin, nELineEnd, eula
  
  ## \brief Compare source file EULA block to tarage EULA license.
  #
  # \param eula   Source file EULA.
  # \param style  Target EULA with added comment style.
  #
  # \return Returns True if EULAs are identical, False otherwise.
  #
  def isEqualEulas(self, eula, style):
    if len(eula) != len(self.m_Eula[style]):
      return False
    i = 0
    while i < len(eula): 
      if eula[i] != self.m_Eula[style][i]:
        return False
      i += 1
    return True
  
  ## \brief Copy source file to temporary while updating the EULA.
  #
  # \param fsrc         Open source file object.
  # \param style        EULA comment style.
  # \param nLineBegin   EULA begin tag line number.
  # \param nLineEnd     EULA end tag line number.
  #
  # \return On success, returns temporary file name.
  #         On failure, an empty string is returned.
  #
  def copySourceToTmp(self, fsrc, style, nLineBegin, nLineEnd):
    tmpfile = fsrc.name + '.tmp'
    try:
      fdst = open(tmpfile, "w")
    except:
      self.iowarning(fdst, "Cannot open temporary before updating.")
      return ""
    fsrc.seek(0)
    n = 1
    # lines before eula
    while n < nLineBegin:
      fdst.write(fsrc.readline())
      n += 1
    # new eula
    fdst.write(self.m_EulaTag[style]['begin'])
    for line in self.m_Eula[style]:
      fdst.write(line)
    fdst.write(self.m_EulaTag[style]['end'])
    # eat old eula
    while n <= nLineEnd:
      fsrc.readline()
      n += 1
    # lines after eula
    for line in fsrc:
      fdst.write(line)
    fdst.close()
    return tmpfile
  
  ## \brief Check and, if needed, update source file EULA.
  #
  # \param f          Open source file object.
  # \param style      Comment style.
  # \param doupdate   Do [not] update.
  # \param doforce    Do [not] force update on good files.
  #
  def updateEula(self, f, style, doupdate, doforce):
    found, nLineBegin, nLineEnd, eula = self.findEula(f)
    #print "(%s, %d, %d) %s" % (style, nLineBegin, nLineEnd, f.name)
    if not found:
      return

    # one-time add of an eula with the given doxygen comment style
    if not self.m_Eula.has_key(style):
      self.addEulaStyle(style)
      #for line in self.m_Eula[style]:
      #  print "%s" % (line),

    # check if eulas are equal
    if self.isEqualEulas(eula, style) and not doforce:
      return

    if doupdate:
      tmpfile = self.copySourceToTmp(f, style, nLineBegin, nLineEnd)
      f.close()
      if not tmpfile:
        return
      try:
        shutil.move(tmpfile, f.name)
      except IOError:
        self.ioerror(f, "Cannot move temporary {0} back to source.", tmpfile)
        return
      print "Updated: {0}".format(f.name)
      self.m_nUpdateCnt += 1
    else:
      print "Update required: {0}".format(f.name)
      self.m_nUpdateCnt += 1
  
  ##
  # \brief Run application on file.
  #    
  # \param kwargs  Optional keyword argument list.
  #
  def runOnFile(self, **kwargs):
    print "EULA:       {0}".format(self.m_EulaLicenseTag)
    print "File:       {0}".format(kwargs['file'])
    print "Scanning..."

    self.m_nUpdateCnt = 0

    fname = kwargs['file']
    if not os.path.isfile(fname):
      self.ioerror(fname, "Does not exist.")
      return
    style = self.classifyFileType(fname)
    if style:
      try:
        f = open(fname, "r")
      except:
        self.ioerror(fname, "Cannot open.")
        return
      self.updateEula(f, style, kwargs['update'], kwargs['force'])
      if not f.closed:
        f.close()

  ## \brief Run application recursively on directory.
  #    
  # \param kwargs  Optional keyword argument list.
  #
  def runOnDirectory(self, **kwargs):
    print "EULA:       {0}".format(self.m_EulaLicenseTag)
    print "Directory:  {0}".format(kwargs['topdir'])
    print "Scanning..."

    self.m_nUpdateCnt = 0

    for root, dirlist, filelist in os.walk(kwargs['topdir']):
      #print "------------------------"
      #print "root: ", root
      for pat in Pat_excludes:
        for dirname in dirlist:
          if fnmatch.fnmatch(dirname, pat):
            dirlist.remove(dirname)
            break

      #print "dirlist: ", dirlist
      for fname in filelist:
        style = self.classifyFileType(fname)
        if style:
          fpath = root + os.path.sep + fname
          try:
            f = open(fpath, "r")
          except:
            print "Warning: {0}: Cannot open.".format(fpath)
            continue
          self.updateEula(f, style, kwargs['update'], kwargs['force'])
          if not f.closed:
            f.close()


  ## \brief Load EULA plaintext file.
  #
  # \param licence  License tag.
  #
  def loadEula(self, license):
    self.m_EulaLicenseTag    = license
    self.m_EulaPlainTextFile = EulaTemplatesRoot + os.path.sep + 'eula_' + \
                                self.m_EulaLicenseTag + '.txt'

    try:
      f = open(self.m_EulaPlainTextFile, "r")
    except:
      self.ioerror(self.m_EulaPlainTextFile,
          "Cannot open EULA plain text file.")
      sys.exit(2)

    self.m_Eula['plaintext'] = f.readlines()

    f.close()

  ## \brief Run application.
  #    
  # \param argv    Optional argument list to override command-line arguments.
  # \param kwargs  Optional keyword argument list.
  #
  def run(self, argv=None, **kwargs):
      
    kwargs = self.getOptions(argv, **kwargs)

    self.loadEula(kwargs['license'])

    if kwargs['file']:
      self.runOnFile(**kwargs)
    else:
      self.runOnDirectory(**kwargs)

    print "Files:      %s" % (self.m_nUpdateCnt)

    return 0


    print "Directory:  %s" % (kwargs['topdir'])
    print "Scanning..."

    self.m_nUpdateCnt = 0

    for root, dirlist, filelist in os.walk(kwargs['topdir']):
      #print "------------------------"
      #print "root: ", root
      for pat in Pat_excludes:
        for dirname in dirlist:
          if fnmatch.fnmatch(dirname, pat):
            dirlist.remove(dirname)
            break
      #print "dirlist: ", dirlist
      for fname in filelist:
        style = self.classifyFileType(fname)
        if style:
          fpath = root + os.path.sep + fname
          try:
            f = open(fpath, "r")
          except:
            print "Warning: %s: Cannot open." % (fpath)
            continue
          self.updateEula(f, style, kwargs['update'])
          if not f.closed:
            f.close()

    print "Files:      %s" % (self.m_nUpdateCnt)

    return 0
  ##


#------------------------------------------------------------------------------
# Main
#------------------------------------------------------------------------------

if __name__ == '__main__':
  initRNMakeEnv()

  print RNMakeEnv

  EulaTemplatesRoot = RNMakeEnv['tool_templates'] + '/eula'

  # create utility application
  app = Application()

  # run application
  sys.exit( app.run() )
