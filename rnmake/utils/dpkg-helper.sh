#!/bin/sh
# Package:  RN Makefile System Utility
# File:     dpkg-helper.sh
# Desc:     Build debian package
# Usage:    dpkg-helper <dev|src|doc> 
#
# /*! \file */
# /*! \cond RNMAKE_DOXY*/


optstr="a:c:d:n:p:t:v:y:"

dist_dir=
deb_arch=
deb_confdir=
deb_name=
deb_prefix=
deb_tmpdir=
deb_version=
pkg_type=

while getopts :${optstr} opt
do
  case $opt in
    a) deb_arch=${OPTARG} ;; 

    c) deb_confdir=${OPTARG} ;;

    d) dist_dir=${OPTARG} ;;

    n) deb_name=${OPTARG} ;;

    p) deb_prefix=${OPTARG} ;;
      
    t) deb_tmpdir=${OPTARG} ;;

    v) deb_version=${OPTARG} ;;

    y) pkg_type=${OPTARG} ;;

    *) echo "rnmake: $0: error: Unknown opt: $opt"; exit 2;;
  esac
done

# because dpkg-deb uses amd64 instead of x86_64
#if [ $deb_arch = "x86_64" ]
#then
  #deb_arch="amd64"
#fi
case $deb_arch in
  x86_64) deb_arch="amd64";;
  i386)   deb_arch="i386";;
  linaro) deb_arch="armhf";;
  odroid) deb_arch="armhf";;

  *)      echo "rnmake does not yet support the requested debian package arch";
          echo "   *" $deb_arch;
          echo ;
          exit 2;;

esac

deb_tmpdir=${deb_tmpdir}-${deb_arch}

echo
echo "Found debian package configuration directoy:" ${deb_confdir} 
echo "   Creating package" ${deb_name}".deb"
echo "   arch =" ${deb_arch}
echo "   vers =" ${deb_version}
echo "   install prefix =" ${deb_prefix}

if [ ! -e $deb_tmpdir ]
then
  mkdir -p $deb_tmpdir/DEBIAN
  mkdir -p $deb_tmpdir/$deb_prefix
fi

cp -r $deb_confdir/* ${deb_tmpdir}/DEBIAN/.

sed_arch="s/@ARCH@/"$deb_arch"/"
sed_version="s/@VERSION@/"$deb_version"/"
sed -e $sed_version -e $sed_arch --in-place $deb_tmpdir/DEBIAN/control
sed -e $sed_version -e $sed_arch --in-place $deb_tmpdir/DEBIAN/postinst

# copying files from dist
case $pkg_type in
  pkgtype-dev) echo "creating dev package"; \
               cp -r ${dist_dir}/bin ${deb_tmpdir}/${deb_prefix}/.;
               cp -r ${dist_dir}/lib ${deb_tmpdir}/${deb_prefix}/.;
               cp -r ${dist_dir}/include ${deb_tmpdir}/${deb_prefix}/.;
               cp -r ${dist_dir}/share ${deb_tmpdir}/${deb_prefix}/.;
               cp -r ${dist_dir}/etc ${deb_tmpdir}/.;;
  
  pkgtype-src) echo "creating src package"; \
               cp -r ${dist_dir}/src ${deb_tmpdir}/${deb_prefix}/.;;

  pkgtype-doc) echo "creating doc package"; \
               cp -r ${dist_dir}/doc ${deb_tmpdir}/${deb_prefix}/.;;

  *)           echo "rnmake does not support the requested debian package type";
               echo "   * " $deb_type
               echo
esac

fakeroot -- dpkg-deb --build $deb_tmpdir 1>/dev/null
mv $dist_dir/tmp/deb/$deb_name-$deb_arch.deb $dist_dir/.

echo

#/*! \endcond RNMAKE_DOXY */
