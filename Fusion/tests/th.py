import threading as thread
import time

def TheTh(args=(), kwargs={}):
  print 'TheTh: hi'
  print 'TheTh:', repr(thread.currentThread().getName())
  time.sleep(3.0)
  print 'TheTh: bye'

def do():
  theTh = thread.Thread(target=TheTh, name='TheTh', kwargs={})
  print 'do:', repr(theTh.getName())
  theTh.start()
  time.sleep(4.0)
  print 'do: join'
  theTh.join()
  print 'do: thread is dead'



