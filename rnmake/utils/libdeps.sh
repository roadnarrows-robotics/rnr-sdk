#!/bin/sh
# Package:  RN Makefile System Utility
# File:     libdeps.sh
# Desc:     Build package library dependency list
# Usage:    libdeps.sh libvpath libs...
# Example:  libdeps.sh ../lib:../dist/dist-<arch>/lib  common foo bar
#
# /*! \file */
# /*! \cond RNMAKE_DOXY*/

libvpath=${1}
shift

for lib in $*
do
  IFS=':'
  for libdir in ${libvpath}
  do
    if [ -f ${libdir}/lib${lib}.so ]
    then
      echo ${libdir}/lib${lib}.so
      break;
    elif [ -f ${libdir}/lib${lib}.a ]
    then
      echo ${libdir}/lib${lib}.a
      break;
    fi
  done
done

exit 0

#/*! \endcond RNMAKE_DOXY */
