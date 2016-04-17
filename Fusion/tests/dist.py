import math

def dist(v1, v2, w=2, mod=10):
  v1 = math.fmod(v1, mod)
  v2 = math.fmod(v2, mod)
  w = float(w)
  mod = float(mod)
  l = math.fmod(v2 - w + mod, mod)
  u = math.fmod(v2 + w, mod)
  print 'w(%7.3f, %7.3f) = [%7.3f, %7.3f] <-- %7.3f' % \
      (math.degrees(v2), math.degrees(w), 
       math.degrees(l), math.degrees(u), math.degrees(v1)),
  if l > u:
    if l <= v1 and v1 <= mod:
      print 'y'
    elif v1 <= u:
      print 'y'
    else:
      print 'n'
  elif l <= v1 and v1 <= u:
    print 'y'
  else:
    print 'n'

def run1():
  d = [
    (0, 9), (0, 8), (0, 7),
    (1, 1), (1, 0), (1, 9), (1, 8), (1, 7),
    (2, 2), (2, 1), (2, 0), (2, 9),
    (5, 6), (5, 7), (5, 8), 
    (9, 9), (9, 0), (9, 1), (9, 2),
    (6, 5), (6, 4), (6, 3), (6, 2), (6, 1), (6, 0)
  ]
  for p in d:
    print p, ':',
    dist(p[0], p[1])


l_coop = math.radians(40.0)

def f_w():
  """ Target interaction kernel.
  """
  w = l_coop / 2.0
  for psi1 in [0, 60, 120, 180, 240, 360]:
    for psi2 in [0, 20, 40, 60, 80, 100, 120, 140, 160, 180, 200, 220, 240, 
                 260, 280, 300, 320, 340, 360]:
      print '(%4d, %4d)' % (psi1, psi2),
      dist(math.radians(psi1), math.radians(psi2), w=w, mod=math.pi*2.0)
