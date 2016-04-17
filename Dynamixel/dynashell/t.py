ccw = [(55, 40), (30, 330), (1,0), (2,359)]
cw = [(40, 55), (330, 30), (0,1), (359,2)]

print "The Data"
print "ccw"
for p in ccw:
  print p[0], p[1]

print
print "cw"
for p in cw:
  print p[0], p[1]

def dp(pc, pp):
  dp = pc - pp
  if abs(dp) > 180:
    if dp < 0:
      dp = 360 + dp
    else:
      dp = dp - 360
  print pc, pp, dp

print
print
print "The Deltas"
print "ccw"
for p in ccw:
  dp(p[0], p[1])

print
print "cw"
for p in cw:
  dp(p[0], p[1])
