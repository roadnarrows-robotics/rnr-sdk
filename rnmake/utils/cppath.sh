#!/bin/sh
# Package:  RN Makefile System Utility
# File:     cppath.sh
# Desc:     Copies path file to destination directory.
# Usage:    cppath.sh <src> <dstdir>
#
# /*! \file */
# /*! \cond RNMAKE_DOXY*/

src=$1
dstdir=$2

if [ "$src" = "" ]
then
  echo "rnmake: $0: error: no source file."
  exit 2
fi
if [ "$dstdir" = "" ]
then
  echo "rnmake: $0: error: no destination directory."
  exit 2
fi
  
if [ -d $src ]
then
  d=$dstdir/$src
  test -d $d || mkdir -p -m 775 $d
else 
  d=$dstdir/$(dirname $src)
  test -d $d || mkdir -p -m 775 $d
  cp -p $src $d/.
fi

#/*! \endcond RNMAKE_DOXY */
