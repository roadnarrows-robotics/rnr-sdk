def cwinstart(callobj, *args, **kwargs):
  print 'cwinstart'
  print '  args', repr(args)
  for arg in args:
    print '    ', arg
  print '  kwargs', len(kwargs)
  for k, v in kwargs.iteritems():
    print '    ', k, v

  w = callobj(*args, **kwargs)

  print '  callobj()->', w

  return w

def cwincall(req1, req2, *args, **kwargs):
  print 'cwincall'
  print '  req1=', req1, 'req2=', req2
  print '  args', repr(args)
  for arg in args:
    print '    ', arg
  print 'kwargs'
  for k, v in kwargs.iteritems():
    print '    ', k, v
  return 'tomorrow'
