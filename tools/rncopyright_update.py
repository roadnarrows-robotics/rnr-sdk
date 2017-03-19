#!/usr/bin/python
################################################################################
#
# Package:  tools
#
# File:     rncopyright_update.py
#
# Usage: rncopyright_update.py [OPTIONS] [<dir> | <file>]
#
# Description:
#   RN package utility.
#
#   Update package source copyright dates to present year.
#
# Author: Robin Knight (robin.knight@roadnarrows.com)
#
# Copyright (C) 2017.  RoadNarrows LLC.
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

import os
import sys
import shutil
import datetime as dt
import fnmatch
import re
import getopt

## Excluded directories
Pat_excludes  = [
    '.svn', 'obj', '.deps', 'dist', 'loc', 'hw', 'gtest', 'tinyxml',
    'build', 'build.*', 'doxy'
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

## File comment style patterns
Pat_slashstar_style = Pat_c_src + Pat_j_src + Pat_doxy_src + Pat_js_src
Pat_hashhash_style  = Pat_py_src
Pat_hashbang_style  = Pat_mk_src
Pat_xml_style       = Pat_xml_src

## Slash-star (c, java, doxygen, php, ...) comment style constructs
ComSlashStarKey     = "/* */"
ComSlashStarStart   = "/*!"
ComSlashStarMiddle  = " *"
ComSlashStarEnd     = " */"

## Hash-hash (python, perl, ...) doxygen comment style constructs
ComHashHashKey      = "##"
ComHashHashStart    = "##"
ComHashHashMiddle   = "##"
ComHashHashEnd      = "##"

## Hash-bang (makefiles, ...) doxygen comment style constructs
ComHashBangKey      = "##!"
ComHashBangStart    = "##!"
ComHashBangMiddle   = "##!"
ComHashBangEnd      = "##!"

## XML (html, xml, ...) comment style constructs
ComXmlKey           = "<!-- -->"
ComXmlStart         = "<!--"
ComXmlMiddle        = " - "
ComXmlEnd           = " -->"

## Doxygen Supported Comments 
CommentStyle = {
    ComSlashStarKey: {
      'start':  ComSlashStarStart,
      'middle': ComSlashStarMiddle,
      'end':    ComSlashStarEnd
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
    }
}

## Minimum copyright year
YearRNCreated       = 2002


##
## \brief Command-line exception class.
##
## Raise usage excpetion.
##
class usage(Exception):

  ##
  ## \brief Constructor.
  ##
  ## \param msg   Error message string.
  ##
  def __init__(self, msg):
    ## error message attribute
    self.msg = msg


##
## \brief Application utilitiy class.
##
class Application():

  ##
  ## \brief Constructor.
  ##
  def __init__(self):
    ## command name
    self._Argv0 = __file__

    self.m_copyright = {}
    self.m_copyright['template'] = [
        "{0} \\copyright\n",
        "{0}   \\h_copy {1}-{2}. RoadNarrows LLC.\\n\n",
        "{0}   http://www.roadnarrows.com\\n\n",
        "{0}   All Rights Reserved\n"
    ]

    ## Current year
    self.m_yearNow = dt.date.today().year

    self.m_nUpdateCnt = 0

  ##
  ## \brief Print usage error.
  ##
  ## \param emsg  Error message string.
  ##
  def printUsageErr(self, emsg):
    """ Print Error Usage Message.
          
        Parameters:
          msg   - Error message string.
    """
    if emsg:
      print "{0}: {1}".format(self._Argv0, emsg)
    else:
      print "{0}: error".format(self._Argv0)
    print "Try '{0} --help' for more information.".format(self._Argv0)

  ##
  ## \brief Print Command-Line Usage Message.
  ##
  def printUsage(self):
    print \
"""

Usage: {0} [OPTIONS] [<dir>]
       {0} [OPTIONS] [<file>]
       {0} --help

Update source file copyrights with this current year.

Description:
  If a directory <dir> is specified, then recursively update all source file
  copyrights with the current year, starting from directory <dir>.
  Only files that have copyrights that differ in the ending year from
  current year will be updated, unless the --force option is specified.
  Source files are defined as:
    C, C++, Java, Python, make files, doxygen, html, xml

  If a regular <file> is specified, then update the file's copyright with the
  current year.

  If neither <dir> nor <file> is specified, the default is '.' - the current
  working directory.

Options and Arguments:
-n, --noupdate            : List files that need updating, but don't update.
-f, --force               : Force update, even if years match. Must be good.
-e, --exclude <dir>       : Exclude directory. May be iterated.

-h, --help                : Display this help and exit.
""".format(self._Argv0)
 
  ##
  ## \brief Print file warning message.
  ##
  ## \param f       Opened file object.
  ## \param warnmsg Warning messages with {n} markup starting at 0.
  ## \param args    Argument list to warnmsg.
  ##
  def warning(self, f, warnmsg, *args):
    print "Warning: {0}:".format(f.name),
    print warnmsg.format(*args)

  ##  
  ## \brief Get command-line options
  ##  
  ## \param argv          Argument list. If not None, then overrides
  ##                      command-line arguments.
  ## \param [out] kwargs  Keyword argument list.  
  ##
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
          sys.exit(0)
    except usage, err:
      self.printUsageErr(err.msg)
      sys.exit(2)

    # update directory or file
    if len(args) > 0:
      if os.path.isdir(args[0]):
        kwargs['topdir'] = args[0]
      else:
        kwargs['file'] = args[0]

    return kwargs

  ##
  ## \brief Classify comment style from file name.
  ##
  ## \param fname   File name.
  ##
  ## \return Comment style.
  ##
  def classifyFileType(self, fname):
    for pat in Pat_slashstar_style:
      if fnmatch.fnmatch(fname, pat):
        return ComSlashStarKey
    for pat in Pat_hashhash_style:
      if fnmatch.fnmatch(fname, pat):
        return ComHashHashKey
    for pat in Pat_hashbang_style:
      if fnmatch.fnmatch(fname, pat):
        return ComHashBangKey
    for pat in Pat_xml_style:
      if fnmatch.fnmatch(fname, pat):
        return ComXmlKey
    return None
  
  ##
  ## \brief Add copyright of the given style to the dictionary of EULAs
  ##
  ## \param style   Comment style
  ##
  def makeCopyrightBlock(self, style, yearBegin, yearEnd):
    self.m_copyright[style] = []
    for line in self.m_copyright['template']:
      self.m_copyright[style].append(
          line.format(CommentStyle[style]['middle'], yearBegin, yearEnd))
  
  ##
  ## \brief Find copyright block in open source file.
  ##
  ## \param f   Open source file object.
  ##
  ## \return
  ## Returns a 4-tuple (found, nCLineBegin, nCLineEnd, copyright) where:\n
  ##  found is True or False if copyright block is found.\n
  ##  nCLineBegin is the line number of the beginning of copyright block.\n
  ##  nCLineEnd is the line number of the end of copyright block.\n
  ##  copyright is the line text holding the copyright year.
  ##
  def findCopyright(self, f):
    nCLineBegin = -1  # doxygen '\par Copyright' or '\copyright' line number
    nCLineYear  = -1  # actual '(C) YEARRANGE. RoadNarrows LLC.' line number
    nCLineEnd   = -1  # end of Copyright paragraph

    fail = (False, -1, -1, "")    # fail tuple

    n = 0   # working line number

    #
    # Search file for copyright block.
    #
    for line in f:
      n += 1

      # copyright begin pattern
      if re.search(r"\\par\s+Copyright|\\copyright", line, re.IGNORECASE):
        nCLineBegin = n
      # copyright year pattern
      elif re.search(r"(\(C\)|h_copy)\s+[0-9,-]+\.*\s+RoadNarrows",
                    line, re.IGNORECASE):
        nCLineYear = n
        copyright  = line  # save line
      # copyright end pattern
      elif re.search(r"All Rights Reserved", line, re.IGNORECASE):
        nCLineEnd = n
        break
      # copyright block is too long
      elif nCLineBegin >= 0 and nCLineEnd == -1 and n - nCLineBegin > 6:
        self.warning(f, "Copyright block starting at line {0} is bogus.",
            nCLineBegin)
        return fail

    #
    # final checks
    #

    # no begin pattern
    if nCLineBegin < 0:
      self.warning(f, "Copyright block is missing or non-standard.")
      return fail
    # no year pattern
    elif nCLineYear < 0:
      self.warning(f, "Copyright year line is missing.")
      return fail
    # no year pattern
    elif nCLineEnd < 0:
      self.warning(f, "Copyright end pattern missing or non-standard.")
      return fail
    # begin pattern found but end position preceeds it
    elif nCLineEnd <= nCLineBegin:
      self.warning(f,
          "Copyright begin pattern at {0} follows end pattern at {1}.",
           nCLineBegin, nCLineEnd)
      return fail
    # year no between begin and end patterns
    elif nCLineYear < nCLineBegin or nCLineYear > nCLineEnd:
      self.warning(f, "Copyright year at {0} not between " \
                 "begin and end patterns {1}-{2}.",
          nCLineYear, nCLineBegin, nCLineEnd)
      return fail
    # good
    else:
      return True, nCLineBegin, nCLineEnd, copyright[0:-1]
  
  ##
  ## \brief Get the source file copyright beginning and ending years.
  ##
  ## \param f         Open source file.
  ## \param copyright Source copyright.
  ##
  ## \return
  ## Returns a 3-tuple (modify, yearBegin, yearEnd) where:\n
  ##  modify is True or False if copyright needs to be modified in the file.\n
  ##  yearBegin is the beginning copyright year.\n
  ##  yearEnd is the ending copyright year.\n
  ##
  def getCopyrightYears(self, f, copyright, doforce):
    m = re.search(r"([12]\d+)[, -]*([12]\d\d\d)?", copyright)
    #print m.groups()
    if m is None:
      self.warning(f,
          "Copyright line specifies no valid year(s)\nLine: {0}",
          copyright)
      return (False, -1, -1)

    try:
      yearBegin = int(m.group(1))
    except:
      self.warning(f, "Copyright year {0} syntax error.",  m.group(1))
      return (False, m_group(1), -1)

    if m.group(2) is not None:
      try:
        yearEnd = int(m.group(2))
      except:
        self.warning(f, "Copyright year {0} syntax error.",  m.group(2))
        return (False, yearBegin, m_group(2))
    else:
      yearEnd = yearBegin

    if yearBegin < YearRNCreated or yearBegin > self.m_yearNow:
      self.warning(f, "Copyright year {0} not in range [{1}, {2}].",
          yearBegin, YearRNCreated, self.m_yearNow)
      return (False, yearBegin, yearEnd)
    if yearEnd < YearRNCreated or yearEnd > self.m_yearNow:
      self.warning(f, "Copyright year {0} not in range [{1}, {2}].",
          yearEnd, YearRNCreated, self.m_yearNow)
      return (False, yearBegin, yearEnd)
    if yearEnd < yearBegin:
      self.warning(f, "Copyright ending year {0} < beginning year {1}.",
          yearEnd, yearBegin)
      return (False, yearBegin, yearEnd)

    if yearEnd < self.m_yearNow or doforce:
      return (True, yearBegin, self.m_yearNow)
    else:
      return (False, yearBegin, yearEnd)
  
  ##
  ## \brief Copy source file to temporary while updating the EULA.
  ##
  ## \param fsrc    Open source file object.
  ## \param style   EULA comment style.
  ## \param n0      EULA begin tag line number.
  ## \param n1      EULA end tag line number.
  ##
  ## \return On success, returns temporary file name.
  ##         On failure, an empty string.
  ## returned.
  ##
  def copySourceToTmp(self, fsrc, style, nLineBegin, nLineEnd):
    tmpfile = fsrc.name + '.tmp'
    try:
      fdst = open(tmpfile, "w")
    except:
      self.warning(fdst, "Cannot open temporary before updating - skipping.")
      return ""
    fsrc.seek(0)
    n = 1
    while n < nLineBegin:
      fdst.write(fsrc.readline())
      n += 1
    for line in self.m_copyright[style]:
      fdst.write(line)
    while n <= nLineEnd:
      fsrc.readline()
      n += 1
    for line in fsrc:
      fdst.write(line)
    fdst.close()
    return tmpfile
  
  ##
  ## \brief Check and, if needed, update source file copyright.
  ##
  ## \param f         Open source file object.
  ## \param style     Comment style.
  ## \param doupdate  Do [not] update.
  ##
  def updateCopyright(self, f, style, doupdate, doforce):
    found, nLineBeg, nLineEnd, copyright = self.findCopyright(f)
    #print "{0}[{1},{2}]: '{3}'".format(f.name, nLineBeg, nLineEnd, copyright)
    if not found:
      return

    modify, yearBegin, yearEnd = self.getCopyrightYears(f, copyright, doforce)
    #print "{0}: modify={1}, years=[{2}, {3}]".format(f.name,
    #    modify, yearBegin, yearEnd)
    if not modify:
      return

    self.makeCopyrightBlock(style, yearBegin, yearEnd)

    if doupdate:
      tmpfile = self.copySourceToTmp(f, style, nLineBeg, nLineEnd)
      f.close()
      if not tmpfile:
        return
      try:
        shutil.move(tmpfile, f.name)
      except IOError:
        self.warning(f, "Cannot move temporary {0} back to source.", tmpfile)
        return
      print "Updated: {0}".format(f.name)
      self.m_nUpdateCnt += 1
    else:
      print "Update required: {0}".format(f.name)
      self.m_nUpdateCnt += 1
  
  ##
  ## \brief Run application on file.
  ##    
  ## \param kwargs  Optional keyword argument list.
  ##
  def runOnFile(self, **kwargs):
    print "Copyright:  {0}".format(self.m_yearNow)
    print "File:       {0}".format(kwargs['file'])
    print "Scanning..."

    fname = kwargs['file']
    if not os.path.isfile(fname):
      print "Error: {0}: Doet not exist.".format(fname)
      return
    style = self.classifyFileType(fname)
    if style:
      try:
        f = open(fname, "r")
      except:
        print "Error: {0}: Cannot open.".format(fpath)
        return
      self.updateCopyright(f, style, kwargs['update'], kwargs['force'])
      if not f.closed:
        f.close()

  ##
  ## \brief Run application recursively on directory.
  ##    
  ## \param kwargs  Optional keyword argument list.
  ##
  def runOnDirectory(self, **kwargs):
    print "Copyright:  {0}".format(self.m_yearNow)
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
          self.updateCopyright(f, style, kwargs['update'], kwargs['force'])
          if not f.closed:
            f.close()

  ##
  ## \brief Run application.
  ##    
  ## \param argv    Optional argument list to override command-line arguments.
  ## \param kwargs  Optional keyword argument list.
  ##
  def run(self, argv=None, **kwargs):
  
    kwargs = self.getOptions(argv, **kwargs)

    if kwargs['file']:
      self.runOnFile(**kwargs)
    else:
      self.runOnDirectory(**kwargs)

    print "Files:      %s" % (self.m_nUpdateCnt)

    return 0
  ##


#------------------------------------------------------------------------------
# Main
#------------------------------------------------------------------------------

# create utility application
app = Application()

# run application
sys.exit( app.run() )
