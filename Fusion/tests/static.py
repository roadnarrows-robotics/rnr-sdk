# test 'static' concepts in python

class C:
  def __init__(self, arg):
    self.which = arg
    print('C(%s)' % repr(self.which))
    #self.v = self.oncebylist()
    self.v = self.oncebydict()
    print('C(%s):staticvars=%s' % (repr(self.which), repr(self.v)))

  # a mutable object like a list element as default argument
  # will act like a static variable since its address is fixed 
  def oncebylist(self, staticvars=[0, 0, 0]):
    if staticvars[0] == 0:
      print('C(%s):setting once' % repr(self.which))
      staticvars[0] = 1
      staticvars[1] = 5
      staticvars[2] = 10
    return staticvars[1:]

  # a mutable object like a dictinary element as default argument
  # will act like a static variable since its address is fixed 
  def oncebydict(self, staticvars={'done_once':0}):
    if not staticvars['done_once']:
      print('C(%s):setting once' % repr(self.which))
      staticvars['done_once'] = 1
      staticvars['static1'] = 50
      staticvars['static2'] = 100
    for k,v in staticvars.items():
      setattr(self, k, v)
    return staticvars

  def what(self):
    print('C(%s):staticvars=%s' % (repr(self.which), repr(self.v)))

  def whatattr(self):
    print('C(%s):' % (repr(self.which)))
    for name in ['static1', 'static2']:
      print('  %s=%s' % (name, getattr(self, name)))
