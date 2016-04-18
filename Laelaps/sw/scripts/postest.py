# python routines to test robot position calculations

# 
# Paths are odometer distances from previous positions
#

import math as m

Tau       = 2.0 * m.pi        # a better pi
Wheelbase = 4.0               # robot wheelbase
CoR       = Wheelbase / 2.0   # robot left-right center of rotation

#
# r = radius of curvature
# theta = degrees of arc
#
def botturn(name, r, theta):
  if theta >= 0.0:
    odleft  = (r-CoR) * theta
    odright = (r+CoR) * theta
  else:
    odleft  = (r+CoR) * -theta
    odright = (r-CoR) * -theta
  return name, odleft, odright

paths = {
  # right angle to the left
  'path1': [
    ("up", 100, 100), botturn("left90", 0, m.pi/2), ("left", 100, 100)],

  # spin left 360
  'spinleft':   [botturn("spin360_left", 0, Tau)],

  # spin right 360
  'spinright':  [botturn("spin360_right", 0, -Tau)],

  # up right at a 45
  'path2': [botturn("right45", 0, -m.pi/4), ("up_right", 100, 100)],

  # diamond closed path
  'diamond': [
    botturn("right45", 0, -m.pi/4),
    ("up_right", 100, 100),
    botturn("right90", 0, -m.pi/2),
    ("dn_right", 100, 100),
    botturn("right90", 0, -m.pi/2),
    ("dn_left", 100, 100),
    botturn("right90", 0, -m.pi/2),
    ("up_left", 100, 100)],

  # 8th of a circle
  'ccw 8th-circle': [botturn("circle(pi/4)", 100.0 * CoR, m.pi/4)],

  # quarter of a circle
  'ccw qtr-circle': [botturn("circle(pi/2)", 100.0 * CoR, m.pi/2)],

  # half circle
  'ccw half-circle': [botturn("circle(pi)", 100.0 * CoR, m.pi)],

  # full circle closed path
  'ccw full-circle': [botturn("circle(2*pi)", 100.0 * CoR, Tau)],

  # two full circles closed path
  'ccw two-circles': [botturn("circle(4*pi)", 100.0 * CoR, 2.0*Tau)],

  # quarter of a circle
  'cw qtr-circle': [botturn("circle(pi/2)", 100.0 * CoR, -m.pi/2)],
}

# current pose+
cur_pose = { }
prev_pose = { }

def resetpose(pose):
  pose['od_left']   = 0.0
  pose['od_right']  = 0.0
  pose['s']         = 0.0
  pose['r']         = 0.0
  pose['c']         = 0.0
  pose['x']         = 0.0
  pose['y']         = 0.0
  pose['theta']     = 0.0

def printpose(name, pose):
  print "%s: (x,y,theta)=(%.3f, %.3f, %.3f), " \
      "s=%.3f, r=%.3f, c=%.3f, " \
      "od=[%.3f, %.3f]" % \
      (name, pose['x'], pose['y'], m.degrees(pose['theta']),
      pose['s'], pose['r'], m.degrees(pose['c']),
      pose['od_left'], pose['od_right'])

def theta(od_left, od_right):
  theta = (od_right - od_left) / Wheelbase
  #return m.fmod(theta, Tau)
  return theta

def pose():
  global cur_pose, prev_pose

  dleft   = cur_pose['od_left'] - prev_pose['od_left']
  dright  = cur_pose['od_right'] - prev_pose['od_right']
  s       = (dright + dleft)/2.0

  theta_j = cur_pose['theta']
  theta_i = prev_pose['theta']
  c       = theta_j - theta_i
  
  # no movement
  if m.fabs(s) <= 1e-4:
    r = 0.0
    x = prev_pose['x']
    y = prev_pose['y']

  # arc path
  elif m.fabs(c) > 1e-4:
    r = s / c
    # let  u = pi/2 - theta
    #     -u = theta - pi/2
    # then:
    #   sin(-u) = -sin(u)
    #   cos(-u) =  cos(u)
    # and
    #   sin(u) = cos(theta)
    #   cos(u) = sin(theta)
    # so
    #   dx = r * (m.cos(theta_j-m.pi/2) - m.cos(theta_i-m.pi/2))
    #   dy = r * (m.sin(theta_j-m.pi/2) - m.sin(theta_i-m.pi/2))
    # can be rewritten as:
    dx = r * (m.sin(theta_j) - m.sin(theta_i))
    dy = r * (m.cos(theta_i) - m.cos(theta_j))

    x = prev_pose['x'] + dx
    y = prev_pose['y'] + dy

  # degenerate case: staight path
  else:
    r = s
    x = prev_pose['x'] + r * m.cos(theta_j)
    y = prev_pose['y'] + r * m.sin(theta_j)

  cur_pose['s'] = s
  cur_pose['r'] = r
  cur_pose['c'] = c

  return x, y

def nav(path):
  global cur_pose, prev_pose
  resetpose(cur_pose)
  resetpose(prev_pose)
  n = 1
  od = [0.0, 0.0]
  for desc, odl, odr in path:
    od[0] += odl
    od[1] += odr
    print "%d. %s: [%.2f, %.2f]" % (n, desc, od[0], od[1])

    prev_pose = cur_pose.copy()
    printpose("  prev_pose", prev_pose)

    cur_pose['od_left']   = od[0]
    cur_pose['od_right']  = od[1]

    cur_pose['theta'] = theta(od[0], od[1])

    cur_pose['x'], cur_pose['y'] = pose()
    cur_pose['theta'] = m.fmod(cur_pose['theta'], Tau)
    printpose("   cur_pose", cur_pose)

    n += 1

def testall():
  for key, path in paths.iteritems():
    print "\n------------------------------------------------------------------"
    print key
    print "------------------------------------------------------------------"
    nav(path)
