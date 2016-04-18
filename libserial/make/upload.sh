#
# Handy little script to upload run-time executables and libraries to an Overo.
#
# Usage: ./upload.sh <addr>
#

_usage="upload.sh <addr>"

# package root
_pkgroot=${PWD%%libserial*}libserial


# load library path
_distdir=${_pkgroot}/dist/dist.overo/

if [ -z "$1" ]
then
  echo "No overo address specified."
  echo ${_usage}
  exit 2
fi

_addr=$1

echo "Pinging ${_addr}"
ping -c 1 ${_addr} >/dev/null

if [ $? != 0 ]
then
  echo "Cannot ping ${_addr}"
  echo ${_usage}
  exit 2
fi
echo

echo "Copying executables to ${_addr}:/usr/local/bin"
scp -r ${_distdir}/bin/* root@${_addr}:/usr/local/bin/.
echo

echo "Copying libraries to ${_addr}:/usr/local/lib"
scp -r ${_distdir}/lib/* root@${_addr}:/usr/local/lib/.
echo

unset _usage _pkgroot _distdir _addr
