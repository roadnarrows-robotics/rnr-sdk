#!/bin/sh
# Package:  RN Makefile System Utility
# File:     doxyindex.sh
# Desc:     Overwrite doxygen's generated index.html with tailored version.
# Usage:    doxyindex.sh -t <title> -h <doxyheader>
#
# /*! \file */
# /*! \cond RNMAKE_DOXY*/

# The options string
optstr="t:h:"

title=
header=

# 
# Get options. Note: first colon says that getopts will not print errors.
#
while getopts :${optstr} opt
do
  case $opt in
    t)  title="$OPTARG" ;;

    h)  header="$OPTARG" ;;

    *) echo "rnmake: $0: error: Unknown opt: $opt"; exit 2;;
  esac
done

shift $(($OPTIND - 1))

if [ "${title}" = "" ]
then
  echo "rnmake: $0: error: No title specfied"
  exit 2
fi

if [ "${header}" = "" ]
then
  echo "rnmake: $0: error: No package header specified"
  exit 2
fi

#
# Add Frame doc type
#
cat <<ENDOFDOC
<!DOCTYPE HTML PUBLIC "-//W3C//DTD HTML 4.01 Frameset//EN">
ENDOFDOC

#
# Header contains the expected html structure.
#  <!DOCTYPE HTML PUBLIC "-//W3C//DTD HTML 4.01 Transitional//EN">
#  <html>
#    <head>
#       header data
#    </head>
#    <body...>
#
# Remove any <DOCTYPE> and <title> elements and the trailing </head> and
# <body> elements.
#
cat ${header} | grep -vi -E "<!DOCTYPE.*>|<title>|</head>|<body.*>"

#
# Add specified title and ending index frameset
#
cat <<ENDOFDOC
    <title>${title}</title>
  </head>
  <frameset cols="250,*">
    <frame src="tree.html" name="treefrm">
    <frame src="main.html" name="basefrm">
    <noframes>
    <a href="main.html">Frames are disabled. Click here to go to the main page.</a>
    </noframes>
  </frameset>
</html>
ENDOFDOC

exit 0

#/*! \endcond RNMAKE_DOXY */
