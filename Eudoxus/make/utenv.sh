#
# Handy little script to fix up environment to run the Eudoxus package
# prior to installing.
#
# Usage: ./utenv.sh
#

# package root
_pkgroot=${PWD%%Eudoxus*}Eudoxus

# architecture to unit test
if [ "$RNMAKE_ARCH_DFT" != "" ]
then
  _utarch=$RNMAKE_ARCH_DFT
else
  _utarch=i386
fi

#
# pathmunge pathvar pathstr
#
pathmunge()
{
  _pathvar="$1"
  if [ -z "$_pathvar" ]
  then
    echo "Error: <pathvar>: Not specified"
    return
  fi
  _pathstr="$2"
  if [ -z "$_pathstr" ]
  then
    echo "Error: <pathstr>: Not specified"
    return
  fi

  _xpend="prepend"
  _pathecho='echo $'$(echo $_pathvar)
  _opath=$(eval $_pathecho)
  unset _npath

  #echo '_pathvar='$_pathvar
  #echo '_pathstr='$_pathstr
  #echo '_opath='$_opath

  if [ -z "$_opath" ]
  then
    _npath="$_pathstr"
  elif ! echo $_opath | /bin/egrep -q "(^|:)$_pathstr($|:)"
  then
    _npath=$_pathstr:$_opath
  fi

  if [ -n "$_npath" ]
  then
    _pathset="export $_pathvar=$_npath"
    $_pathset
    #echo '_npath='$(eval $_pathecho)
  fi

  unset _pathvar _pathstr _xpend _pathecho _opath _npath _pathset _optarg _opt
}

# load library path
_distlib_gst_path=${_pkgroot}/dist/dist.${_utarch}/lib/gst
_distlib_path=${_pkgroot}/dist/dist.${_utarch}/lib:${_distlib_gst_path}

pathmunge GST_PLUGIN_PATH ${_distlib_gst_path}

# add made libraries to search path
pathmunge LD_LIBRARY_PATH ${_distlib_path}

# uncomment the following line to to enable package python modules
pathmunge PYTHONPATH ${_pkgroot}/sw/pyModules/modules

unset _utarg _pkgroot _distlist_bath _py_path
