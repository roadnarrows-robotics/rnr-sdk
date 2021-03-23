#!/bin/sh
# Package:  RN Makefile System Utility
# File:     tarball-src-filter.sh
# Desc:     Filters package, excluding all non-source files
# Usage:    tarball-src-filter.sh <pkgroot>
#
# /*! \file */
# /*! \cond RNMAKE_DOXY*/

pkgroot="${1}"

# 
# Recurse through directories and filter out "non-source" files and directories.
# In awk, regular expression rules are compared against the input records until
# one fires which will execute the associated action. Actions are:
#   'next' skips current record (i.e. filter out from stream)
#   'print $0' prints the line (i.e. allow).
# Note: Order is important. The last record allows all unfiltered lines to be
#       included. Keep last.
#
find ${pkgroot} -print | \
gawk '
  /\.git/             { next }
  /\.gitignore/       { next }
  /\.svn/             { next }
  /\.deps/            { next }
  /\/obj/             { next }
  /\/dist/            { next }
  /\/loc/             { next }
  /\/build/           { next }
  /\/hw/              { next }
  /\/fw/              { next }
  /\/os/              { next }
  /\/3rdparty/        { next }
  /docs\/doxy/        { print $0 }
  /docs\/images/      { next }
  /docs\/.*\.doxy/    { print $0 }
  /docs\/.*/          { next }
  /docs/              { next }
  /\.exe/             { next }
  /\.a/               { next }
  /\.so/              { next }
  /\.o/               { next }
  /\.out/             { next }
  /\.log/             { next }
  /\__pycache__/      { next }
  /\.pyc/             { next }
  /\.pyo/             { next }
  /\.done/            { next }
  /.*/                { print $0 }'


# The old way - too limited
#find ${pkgroot} \( \
#  -regex "${pkgroot}/docs/[^di].*" -or \
#		-wholename "${pkgroot}/hw" -or -name hw -or \
#		-wholename "${pkgroot}/fw" -or -name fw -or \
#		-wholename "${pkgroot}/os" -or -name os -or \
#		-wholename "${pkgroot}/dist" -or -name dist -or \
#		-wholename "${pkgroot}/loc" -or -name loc -or \
#		-name '*.svn*' -or -wholename '.svn' -or \
#		-wholename '*.deps*' -or -name '.deps' -or \
#		-wholename '*obj*' -or -wholename 'obj' -or -wholename '*.o' -or \
#		-wholename '*.out' -or -wholename '*.log' -or \
#		-wholename '*.pyc' -or -wholename '*.pyo' \
#	  \) -prune -or -print | \
