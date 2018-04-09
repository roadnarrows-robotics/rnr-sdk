#!/bin/bash
#
# Determine the status all RNR SDK packages.
#
# Usage: rnsdk_git_status.sh

declare -r rnrdistro="rnr-sdk"
declare -r rnrroot="/prj/pkg"
declare -A modpkgs                # associative array

buildmodlist()
{
  cd ${rnrroot}
  mods=$(git status --porcelain -uno)
  #echo "${mods}"
  if [ -z "${mods}" ]
  then
    return
  fi
  while read code path rest
  do
    pkg=${path%%/*}
    #echo "code=${code}, path=${path}, pkg=${pkg}"
    modpkgs[${pkg}]=${code}
    #echo ${modpkgs[@]}
  done <<<"${mods}"

  #echo ${!modpkgs[@]}
  #echo ${modpkgs[@]}
}

echo "# +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++"
echo "# RNR repo:   ${rnrdistro}"
echo "# Workspace:  ${rnrroot}"
echo "# Date:       $(date)"
echo "# +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++"

buildmodlist

for pkg in ${rnrroot}/*
do
  if [ ! -d ${pkg} ]
  then
    continue
  fi

  cd ${pkg}

  pkgbase=$(basename ${pkg})
  printf "%-30s " ${pkgbase}

  if [ ! -z "${modpkgs[${pkgbase}]}" ]
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
