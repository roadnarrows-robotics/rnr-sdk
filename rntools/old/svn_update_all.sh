#!/bin/sh
# Update all packages found in /prj/pkg
#

rnrdistro="/prj/pkg"

echo "+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++"
echo " RNR SVN Packages: ${rnrdistro}"
echo "+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++"

for pkg in ${rnrdistro}/*
do
  cd ${pkg}
  if [ ! -d ".svn" ]
  then
    continue
  fi
  echo
  echo "----------------------------------------------------------------------"
  echo "Updating package ${pkg}"
  echo "----------------------------------------------------------------------"
  svn update .
done
