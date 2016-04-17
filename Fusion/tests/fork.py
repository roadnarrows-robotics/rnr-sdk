import os
import sys
import time


glob = 'hello, i am global'
childpid = 0
gdata0 = 'notinit'
gdata1 = -1

def me():
  pid = os.fork()

  if pid == 0:
    print 'child here'

    loc = 'and this is childs play'

    print 'c', glob
    print loc

    time.sleep(5)

    print 'goodbye'

  else:
    print 'parent here'

    loc = 'but why am i here?'

    print 'p', glob
    print loc

    time.sleep(8)

    print 'adieu'

def start():
  global childpid
  global gdata0, gdata1

  ldata0 = 'localdata1'
  ldata1 = 99

  gdata0 = 'globaldata1'
  gdata1 = 99

  #sshargs = '%s, %d' % (ldata0, ldata1)
  sshargs = 'f.gdata0, f.gdata1'
  cmd =  'python -c "import Fusion.tests.fork as f; f.child(%s);"' % sshargs
  childin, childout = os.popen2(cmd, 'w') #, 0)

  print 'parent: isatty(childin)', childin.isatty()
  print 'parent: isatty(childout)', childout.isatty()

  print 'parent: started'
  n = 0
  while n < 10:
    line = childout.readline()
    print 'parent: rcvd %s' % repr(line)
    if line == 'quit\n':
      print 'parent: exiting'
      break
    n += 1

def child(arg1, arg2):
  global childpid

  childpid = os.getpid()

  fp = open('/tmp/child.out', 'w')
  print >>fp, 'time=%.2f' % time.time()
  print >>fp, 'pid=%d' % childpid

  print 'child: child pid=%d' % childpid
  print 'child: arg1=%s' % repr(arg1)
  print 'child: arg2=%s' % repr(arg2)
  time.sleep(1)
  print 'quit'
  print 'child: exiting'
  print >>fp, 'child: exiting'


if __name__ == '__main__':
  #me()
  #time.sleep(12)
  start()
