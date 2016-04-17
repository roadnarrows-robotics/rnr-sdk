#!/bin/sh
# Package:  RN Makefile System Utility
# File:     doinstall.sh
# Desc:     Install files for source directory to destination directory
# Usage:    doinstall.sh [-s] mode source_dir dest_dir
#
# /*! \file */
# /*! \cond RNMAKE_DOXY*/

# The options string
optstr="s"

# Option defaults
silent=0

# Positional arguments
mode=
srcdir=
dstdir=

# 
# Get options. Note: first colon says that getopts will not print errors.
#
while getopts :${optstr} opt
do
  case $opt in
    s)  silent=1 ;;

    *) echo "rnmake: $0: error: Unknown opt: $opt"; exit 2;;
  esac
done

shift $(($OPTIND - 1))

mode=${1}
srcdir=${2}
dstdir=${3}

if [ "$mode" = "" ]
then
  echo "rnmake: $0: error: No file creation mode specified."
  exit 2
fi

if [ "$srcdir" = "" ]
then
  echo "rnmake: $0: error: No source directory specified."
  exit 2
fi

if [ "$dstdir" = "" ]
then
  echo "rnmake: $0: error: No destination install directory specified."
  exit 2
fi

install -d -p -m 775 ${dstdir}

cd ${srcdir} >/dev/null

#
# Install files
#
find . -type f -o -type l | \
grep -v \.svn | \
while read srcpath
do
  # strip leading './'
	src="${srcpath##./}"
  # destination [sub]directory name
  dname=${dstdir}/$(dirname ${src})
  # create directory if it does not exist
  if [ ! -d ${dname} ]
  then
    install -d -p -m 775 ${dname}
  fi
  # destination file name
	dst="${dstdir}/${src}"
  if [ $silent = 0 ]
  then 
	  echo "  ${dst}"
  fi
  # 'install' symlink file
  if [ -h ${srcpath} ]
  then
    lnk=$(readlink ${srcpath})
    cd ${dname} >/dev/null
    fname=$(basename ${srcpath})
    if [ ! -f ${fname} ]
    then
      ln -s ${lnk} ${fname}
    fi
    cd - >/dev/null
  # install regular file with given permissions
  else
    install -p -m ${mode} ${srcpath} ${dst}
  fi
done

#/*! \endcond RNMAKE_DOXY */
