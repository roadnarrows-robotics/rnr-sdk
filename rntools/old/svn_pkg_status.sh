#!/bin/sh
# Print versions for all packages found in /prj/pkg
#

rnrdistro="/prj/pkg"

echo "# +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++"
echo "#  RNR SVN Packages: ${rnrdistro}"
echo "#  $(date)"
echo "# +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++"

for pkg in ${rnrdistro}/*
do
  cd ${pkg}
  if [ ! -d ".svn" ]
  then
    continue
  fi
  printf "%-30s " ${pkg}
  svn status | grep "^[ \t]*[^\?]" >/dev/null
  if [ "$?" -eq 0 ]
  then
    printf "M "
  else
    printf "  "
  fi
  if [ ! -f "make/Pkg.mk" ]
  then
    printf "\n"
    continue
  fi
  cat make/Pkg.mk | \
  gawk ' 
    BEGIN { maj = 0; min = 0; rel = 0; year = 0 }
    /^[ \t]*PKG_VERSION_MAJOR[ \t]*=[ \t]*/ { maj = $3 }
    /^[ \t]*PKG_VERSION_MINOR[ \t]*=[ \t]*/ { min = $3 }
    /^[ \t]*PKG_VERSION_RELEASE[ \t]*=[ \t]*/ { rel = $3 }
    /^[ \t]*PKG_VERSION_DATE[ \t]*=[ \t]*/ { year = $3 }
    END { print year "  " maj "." min "." rel } '
done
