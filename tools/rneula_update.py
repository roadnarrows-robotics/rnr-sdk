#!/usr/bin/python
################################################################################
#
# Package:  tools
#
# File:     rneula_update.py
#
# Usage: rneula_update.py [OPTIONS] <license> [<dir>]
#
# Version:
#   $LastChangedDate: 2015-04-13 16:03:38 -0600 (Mon, 13 Apr 2015) $
#   $Rev: 3934 $
#
# Description:
#   RN package utility.
#
#   Update package source file EULA's.
#
# Author: Robin Knight (robin.knight@roadnarrows.com)
#
# Copyright (C) 2013.  RoadNarrows LLC.
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
import fnmatch
import re
import getopt

## EULA templates root directory
EulaTemplatesRoot = '/prj/pkg/tools/templates/eula'

EulaTagBegin  = '@EulaBegin@'
EulaTagEnd    = '@EulaEnd@'

## Excluded directories
Pat_excludes  = ['.svn', 'obj', '.deps', 'dist', 'loc', 'hw']

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
Pat_hash_style      = Pat_mk_src + Pat_py_src
Pat_xml_style       = Pat_xml_src

## Slash-star (c, java, doxygen, php, ...) comment style constructs
ComSlashStarKey     = "/* */"
ComSlashStarStart   = "/*"
ComSlashStarMiddle  = " * "
ComSlashStarEnd     = " */"

## Hash (makefiles, python, perl, ...) comment style constructs
ComHashKey          = "#"
ComHashStart        = "#"
ComHashMiddle       = "# "
ComHashEnd          = "#"

## XML (html, xml, ...) comment style constructs
ComXmlKey           = "<!-- -->"
ComXmlStart         = "<!--"
ComXmlMiddle        = " - "
ComXmlEnd           = " -->"

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

    ## EULA license tag
    self.m_EulaLicenseTag = None

    ## EULA plain text filename
    self.m_EulaPlainTextFile = None

    ## Dictionary of EULAs
    self.m_Eula = { }

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
      print "%s: %s" % (self._Argv0, emsg)
    else:
      print "%s: error" % (self._Argv0)
    print "Try '%s --help' for more information." % (self._Argv0)

  ##
  ## \brief Print Command-Line Usage Message.
  ##
  def printUsage(self):
    print \
"""
usage: %s [OPTIONS] <license> [<dir>]

     %s --help

Recursively update all source files with the EULA of the specified <license>,
starting from directory <dir>. Default for <dir> is '.' - the current working
directory. Only files that differ in license will be updated. Source files are
defined as:
  C, C++, Java, Python, make files, doxygen, html, xml

Supported licenses can be found in %s.

Options and arguments:
-n, --noupdate            : List files that need updating, but don't update.

-h, --help                : Display this help and exit.
"""  % (self._Argv0, self._Argv0, EulaTemplatesRoot)
 
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

    # parse command-line options
    try:
      try:
        opts, args = getopt.getopt(argv[1:], "?hn",
            ['help', 'noupdate', ''])
      except getopt.error, msg:
        raise usage(msg)
      for opt, optarg in opts:
        if opt in ('-n', '--noupdate'):
          kwargs['update'] = False
        elif opt in ('-h', '--help', '-?'):
          self.printUsage()
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
      kwargs['topdir'] = arg[1]
    else:
      kwargs['topdir'] = '.'

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
    for pat in Pat_hash_style:
      if fnmatch.fnmatch(fname, pat):
        return ComHashKey
    for pat in Pat_xml_style:
      if fnmatch.fnmatch(fname, pat):
        return ComXmlKey
    return None
  
  ##
  ## \brief Add EULA of the given style to the dictionary of EULAs
  ##
  ## \param style   Comment style
  ##
  def addEula(self, style):
    if style == ComSlashStarKey:
      block_start   = ComSlashStarMiddle #ComSlashStarStart
      block_middle  = ComSlashStarMiddle
      block_end     = ComSlashStarMiddle #ComSlashStarEnd
    elif style == ComHashKey:
      block_start   = ComHashMiddle #ComHashStart
      block_middle  = ComHashMiddle
      block_end     = ComHashMiddle #ComHashEnd
    elif style == ComXmlKey:
      block_start   = ComXmlMiddle #ComXmlStart
      block_middle  = ComXmlMiddle
      block_end     = ComXmlMiddle #ComXmlEnd
    else:
      print "Error: %s: unsupported comment style" % (sytle)
      sys.exit(4)
    self.m_Eula[style] = []
    self.m_Eula[style].append(block_start+'\n')
    for line in self.m_Eula['plaintext']:
      self.m_Eula[style].append(block_middle+line)
    self.m_Eula[style].append(block_end+'\n')
  
  ##
  ## \brief Find EULA block in open source file.
  ##
  ## \param f   Open source file object.
  ##
  ## \return Returns a 4-tuple (found, n0, n1, eula) where\n
  ## found True or False, n0 is the line number of the begin tag, n1 is the line
  ## number of the end tag, and eula is the block of EULA lines found in between
  ## the tags.
  ##
  def findEula(self, f):
    n0    = -1      # begin tag line number
    n1    = -1      # end tag line number
    eula  = []      # eula block
    n     = 0       # working line number
    for line in f:
      # begin tag
      if re.search(EulaTagBegin, line):
        n0 = n
      # end tag
      elif re.search(EulaTagEnd, line):
        n1 = n
        break
      # eula line
      elif n0 >= 0:
        # eula too long
        if n-n0 > 100:
          print "Warning: %s: EULA block starting at line %d is too long or " \
              "'%s' tag missing." % (f.name, n0, EulaTagEnd)
          return False, -1, -1, []
        # add line to eula block
        else:
          eula.append(line)
      n += 1
    # begin tag but no end tag
    if n0 >= 0 and n1 < 0:
      print "Warning: %s: '%s' tag missing." % (f.name, EulaTagEnd)
      return False, -1, -1, []
    # begin tag but end tag preceeds it
    elif n0 >= 0 and n1 <= n0:
      print "Warning: %s: '%s' tag found at or before '%s' tag." % \
          (f.name, EulaTagEnd, EulaTagBegin)
      return False, -1, -1, []
    # end tag but no begin tag
    elif n1 >= 0 and n0 < 0:
      print "Warning: %s: '%s' tag missing." % (f.name, EulaTagBegin)
      return False, -1, -1, []
    # no eula found (ok)
    elif n0 < 0 and n1 < 0:
      return False, -1, -1, []
    # get a good eula block
    else:
      return True, n0, n1, eula
  
  ##
  ## \brief Compare source file EULA block to tarage EULA license.
  ##
  ## \param eula    Source file EULA.
  ## \param style   Target EULA with added comment style.
  ##
  ## \return Returns True if EULAs are identical, False otherwise.
  ## returned.
  ##
  def compareEulas(self, eula, style):
    if len(eula) != len(self.m_Eula[style]):
      return False
    i = 0
    while i < len(eula): 
      if eula[i] != self.m_Eula[style][i]:
        return False
      i += 1
    return True
  
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
  def copySourceToTmp(self, fsrc, style, n0, n1):
    tmpfile = fsrc.name + '.tmp'
    try:
      fdst = open(tmpfile, "w")
    except:
      print "Warning: %s: Cannot open temporary before updating - skipping." % \
          (tmpfile)
      return ""
    fsrc.seek(0)
    n = 0
    while n <= n0:
      fdst.write(fsrc.readline())
      n += 1
    for line in self.m_Eula[style]:
      fdst.write(line)
    while n < n1:
      fsrc.readline()
      n += 1
    for line in fsrc:
      fdst.write(line)
    fdst.close()
    return tmpfile
  
  ##
  ## \brief Check and, if needed, update source file EULA.
  ##
  ## \param f         Open source file object.
  ## \param style     Comment style.
  ## \param doupdate  Do [not] update.
  ##
  def updateEula(self, f, style, doupdate):
    found, n0, n1, eula = self.findEula(f)
    if not found:
      return
    #print "(%s, %d, %d) %s" % (style, n0, n1, f.name)
    if not self.m_Eula.has_key(style):
      self.addEula(style)
      #for line in self.m_Eula[style]:
      #  print "%s" % (line),
    if self.compareEulas(eula, style):
      return
    if doupdate:
      tmpfile = self.copySourceToTmp(f, style, n0, n1)
      if not tmpfile:
        return
      try:
        ftmp = open(tmpfile, "r")
      except:
        print "Warning: %s: Cannot open temporary." % (tmpfile)
        return
      f.close()
      try:
        f = open(f.name, "w")
      except:
        print "Error: %s: Cannot open to write updates - aborting." % (f.name)
        sys.exit(8)
      for line in ftmp:
        f.write(line)
      f.close()
      ftmp.close()
      os.remove(tmpfile)
      print "Updated:    %s" % (f.name)
      self.m_nUpdateCnt += 1
    else:
      print "Update required: %s" % (f.name)
      self.m_nUpdateCnt += 1
  
  ##
  ## \brief Run application.
  ##    
  ## \param argv    Optional argument list to override command-line arguments.
  ## \param kwargs  Optional keyword argument list.
  ##
  def run(self, argv=None, **kwargs):
  
    kwargs = self.getOptions(argv, **kwargs)

    self.m_EulaLicenseTag    = kwargs['license']
    self.m_EulaPlainTextFile = EulaTemplatesRoot + os.path.sep + 'eula_' + \
                                self.m_EulaLicenseTag + '.txt'

    try:
      f = open(self.m_EulaPlainTextFile, "r")
    except:
      print "Error: %s: Cannot open." % (self.m_EulaPlainTextFile)
      sys.exit(2)

    self.m_Eula['plaintext'] = f.readlines()

    f.close()

    print "EULA:       %s" % (self.m_EulaLicenseTag)
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

# create utility application
app = Application()

# run application
sys.exit( app.run() )
