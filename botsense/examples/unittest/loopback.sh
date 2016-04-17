#!/bin/bash
#
# Quick, down and dirty test of bsProxy server using loopbacks.
#
# Hard coded message format:
#
# HDR
# ---
#  2  magic  aa aa
#  1  tid    00
#  1  hnd    fe           = 254
#  2  msgid  00 03
#  2  blen   00 1f        = 31
# --
#  8 byte
# 
# BODY
# ---
#  2 msgid  00 03
#  1 fcnt   01
#  1 fid    01
#  1 ftype  73            = 's'
#  1 flen   19            = 25
# 25 fval   68 65 6c 6c 6f 20 30 20 66 72
#           6f 6d 20 6c 6f 6f 70 62 61 63
#           6b 2e 73 68 0a  = "hello 0 from loopback.sh\n"
#
# --
# 31 bytes

MAGIC='\xaa\xaa'
HND_SERVER='\xfe'
REQ_LOOPBACK='\x00\x03'

FIXED="${REQ_LOOPBACK}\x01"
FID='\x01'
FTYPE='s'

cnt=0

while :
do
  sleep 1

  cdata="hello ${cnt} from loopback.sh"     # loop this back
  let len=${#cdata}+1                       # length including newline
  flen="\x"$(printf "%02x" ${len})          # hex field length

  tid="\x"$(printf "%02x" ${cnt})           # hex tid
  let len=3+3+${len}                        # total body length
  blen="\x00\x"$(printf "%02x" ${len})      # hex body length

  hdr="${MAGIC}${tid}${HND_SERVER}${REQ_LOOPBACK}${blen}"
  body="${FIXED}${FID}${FTYPE}${flen}${cdata}"

  echo -e "${hdr}${body}" >&3
  read line <&3

  #printf "msgid=0x%02x, tid=%u, blen=%u: " ${line:0:1} ${line:1:1} ${line:2:1}
  echo ${line:11}

  let cnt=${cnt}+1
done 3<>/dev/tcp/127.0.0.1/9195
