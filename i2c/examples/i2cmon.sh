#
# Example Shell Program using the I2C commands.
#
# Usage: i2cmon.sh addr readlen byte1 [byte2 ...]
#
addr=$1
readlen=$2
shift 2

if ! ./i2ccheck --address "$addr"
then
  echo "I2C device $addr not found" >&2
  ./i2cscan --verbose >&2
  exit 2
fi

while :
do
  rsp=$(./i2ctrans --fd 3 --address $addr --count $readlen $*)
  echo $rsp
  sleep 1
done 3<>/dev/i2c/0

