#
# File:
#   rnenv.sh
#
# Usage:
#   source rnenv.sh
#
# Description:
#   Assign RN Make System variables from mash up of command-line options and
#   environment variables. Command-line options take precedence.
#
#   Variables:
#     rnenvLongOpts     RN environment standard long options.
#     rnenvShortOpts    RN environment standard short options.
#     rnmakeRoot        Either --rnmake=val or RNMAKE_ROOT.
#     rnmakeArch        Either --arch=val or RNMAKE_ARCH_DFT.
#     rnmakeXPrefix     Either --xprefix=val or RNMAKE_INSTALL_XPREFIX.
#     rnmakePrefix      Either --prefix=val or RNMAKE_INSTALL_PREFIX.
#     rnmakeRealPrefix  Calculated from rnmakeXPrefix, rnmakePrefix,
#                       and rnmakeArch values.
#     rnmakeWorkspace   Either --workspace=val or RNMAKE_WORKSPACE
#     rnmakeVars        RN Make System make name=value list.
#
#   Functions:
#     rnenvGetOpts()    Get command-line standard options.
#     rnenvParseOpts()  Parse command-line standard options.
#

#echo "DBG: rnenv.sh" >&2

# standard rnmake command-line options
rnenvLongOpts="rnmake:,arch:,xprefix:,prefix:,workspace:"
rnenvShortOpts=""

# defaults
rnmakeRoot=${RNMAKE_ROOT}
rnmakeArch=${RNMAKE_ARCH_DFT}
rnmakeXPrefix=${RNMAKE_INSTALL_XPREFIX}
rnmakePrefix=${RNMAKE_INSTALL_PREFIX}
rnmakeRealPrefix=
rnmakeWorkspace=${RNMAKE_WORKSPACE}
rnmakeVars=

# rnenvGetOpts cmd positional_params
rnenvGetOpts()
{
  # get the rnmake specific options
  OPTS=$(getopt --name $0 -o "${rnenvShortOpts}" --long "${rnenvLongOpts}" -- "${@}")

  if [ $? != 0 ]
  then
    printf "$0: Failed option parsing.\n" >&2
    return 2
  fi
  echo "${OPTS}"
  eval set -- "${OPTS}"
  #echo "DBG: ${OPTS}" >&2
  return 0
}

# rnenvParseOpts cmd positional_params
rnenvParseOpts()
{
  while true
  do
    case "$1" in
      --rnmake)     rnmakeRoot=$2;
                    rnmakeVars="${rnmakeVars} rnmake=${rnmakeRoot}"
                    shift 2;;
      --arch)       rnmakeArch=$2;
                    rnmakeVars="${rnmakeVars} arch=${rnmakeArch}"
                    shift 2;;
      --xprefix)    rnmakeXPrefix=$2;
                    rnmakeVars="${rnmakeVars} xprefix=${rnmakeXPrefix}"
                    shift 2;;
      --prefix)     rnmakePrefix="$2";
                    rnmakeVars="${rnmakeVars} prefix=${rnmakePrefix}"
                    shift 2;;
      --workspace)  rnmakeWorkspace="$2";
                    shift 2;;
      --) shift; break;;
      *) break;;
    esac
  done

  # must be set
  if [ -z "${rnmakeRoot}" ]
  then 
    printf "$0: Error: RNMAKE_ROOT not set." >&2
    exit 4
  fi

  # default if not set
  if [ -z "${rnmakeArch}" ]
  then 
    rnmakeArch=x86_64
  fi

  # real install prefix
  if [ ! -z "${rnmakePrefix}" ]
  then
    rnmakeRealPrefix="${rnmakePrefix}"
  elif [ ! -z "${rnmakeXPrefix}" ]
    rnmakeRealPrefix="${rnmakeXPrefix}/${rnmakerRch}"
  else
    rnmakeRealPrefix="${HOME}/xinstall/${rnmakerRch}"
  fi
}
