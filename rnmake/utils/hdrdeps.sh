#!/bin/sh
# Package:  RN Makefile System Utility
# File:     hdrdeps.sh
# Desc:     Build package header dependency list
# Usage:    hdrdeps.sh -c <depcmd> -f <depfile> [-o <objdir>] \
#                   [-I<incdir> -I<incdir> ...] sources
# Example: 
#   hdrdeps.sh -c "gcc -M" -f .deps/deps.<arch> -o obj/obj-<arch> \
#              -d "-DFOO -UBAR" -I. -I../include foo.c bar.cxx
#
# /*! \file */
# /*! \cond RNMAKE_DOXY*/

# The options string
optstr="c:f:o:d:I:"

depcmd=
depfile=
objdir=
cppflags=
includes=

# 
# Get options. Note: first colon says that getopts will not print errors.
#
while getopts :${optstr} opt
do
  case $opt in
    c)  depcmd="$OPTARG" ;;

    f)  depfile="$OPTARG" ;;

    o)  objdir="$OPTARG"/ ;;

    d)  cppflags="$OPTARG" ;;

    I)  includes="${includes} -I$OPTARG" 
        ;;
    *) echo "rnmake: $0: error: Unknown opt: $opt"; exit 2;;
  esac
done

shift $(($OPTIND - 1))

if [ "$depcmd" = "" ]
then
  echo "rnmake: $0: error: No dependency command specfied"
  exit 2
fi

if [ "$depfile" = "" ]
then
  echo "rnmake: $0: error: No output dependency file specfied"
  exit 2
fi

# Header
echo '# Dependencies'  > ${depfile}
echo '#' $(date) >> ${depfile}
echo ' ' >> ${depfile}

# Dependencies
if [ "$*" != "" ]
then
  #echo ${depcmd} ${cppflags} $includes $*

  # Make dependencies
  ${depcmd} ${cppflags} $includes $*  >> ${depfile}

  # Preface object dependency lines with object directory
  sed -i -e "s%^.*\.o:%${objdir}&%" ${depfile}
fi

exit 0

#/*! \endcond RNMAKE_DOXY */
