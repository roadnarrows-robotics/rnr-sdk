import math

smax = 1000.0
rmax = 200.0
k = 1.0

def s(r):
  global k, smax, rmax
  if r <= 0.0:
    return 0.0
  elif r >= rmax:
    return rmax
  s = ((r / rmax) * smax) * math.exp(k * (r / rmax - 1.0))
  return s

def r(s):
  # Newton's method
  # c(s) = r * exp(u)  ==> f(r) = r * exp(u) - c(s)
  global k, smax, rmax
  if s <= 0.0:
    return 0.0
  elif s >= smax:
    return smax
  r0 = rmax / 2.0    # starting approximation
  cs = (s * rmax) / smax  # c(s)
  print 'cs', cs
  du = k / rmax           # du/dr
  n = 0
  print '>', r0
  while n < 15:
    u = k * (r0 / rmax - 1.0) # u(r)
    eu = math.exp(u)          # exp(u)
    r1 = r0 - (r0 * eu - cs) / (eu * (1.0 + r0 * du)) # r0 - f(r0)/f'(r0)
    print n, r1
    if math.fabs(r1-r0) < 0.001:
      return r1
    r0 = r1
    n += 1
  print n, r1
  return r1
