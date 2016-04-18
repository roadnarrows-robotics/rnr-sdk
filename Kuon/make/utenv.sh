#
# Handy little script to fix up environment to run the package apps
# prior to installing.
#
# Usage: ./utenv.sh
#

# package root
_pkgroot=${PWD%%Kuon*}Kuon

# architecture to unit test
if [ "$RNMAKE_ARCH_DFT" != "" ]
then
  _utarch=$RNMAKE_ARCH_DFT
else
  _utarch=i386
fi


# load library path
_distlib_path=${_pkgroot}/dist/dist.${_utarch}/lib:${_pkgroot}/dist/dist.${_utarch}/lib/botsense

# add made libraries to search path
pathmunge LD_LIBRARY_PATH ${_distlib_path}

# python module search path
_py_path=${_pkgroot}/bsPython/modules/BotSense

# add made libraries to search path
pathmunge PYTHONPATH ${_py_path}

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

unset _utarg _pkgroot _distlist_bath _py_path
