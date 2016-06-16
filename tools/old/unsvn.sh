#!/bin/bash

#echo "Clobbering"
#make clobber

echo "Removing auto-generated header files"
rm -fr include/arch
rm include/version.h

echo "Removing doxygen log files"
rm doxy.*.log

echo "Removing loc directory"
rm -fr loc

echo "Removing dist directory"
rm -fr dist

echo "Removing 3rdparty files and directories"
rm -fr 3rdparty/*

echo "Removing hw directory"
rm -fr hw

echo "Removing .svn directories"
find . -name '.svn' | xargs rm -fr

echo "Removing .dep directories"
find . -name '.deps' | xargs rm -fr

echo "Removing obj directories"
find . -name 'obj' | xargs rm -fr

echo "Removing obj files"
find . -name '*.o' | xargs rm

echo "Removing pyc files"
find . -name '*.pyc' | xargs rm
