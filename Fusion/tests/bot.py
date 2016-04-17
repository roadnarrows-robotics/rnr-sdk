import Fusion.Khepera.KheCmd.KheCmdBase as KheBase
import math

twopi = math.pi * 2.0
rad30 = math.radians(30.0)
rad45 = math.radians(45.0)

wb = float(KheBase.KheWheelBase)

def theta(odl, odr):
  """ odometer left and right readings (mm) """
  r = (odr - odl) / wb
  #print 'raw ratio', r
  if r > 1.0:
    qspins = math.floor(r)  # ccw quater spins
    offset = math.radians(qspins * 90.0)
    r -= qspins
  elif r < -1.0:
    qspins = -math.floor(math.fabs(r))  # cw quater spins
    offset = math.radians(qspins * 90.0)
    r -= qspins
  else:
    offset = 0.0
  #print 'adj ratio', r
  ftheta = math.asin(r)
  #print 'raw theta', ftheta
  ftheta = math.fmod(ftheta+offset, twopi)
  if ftheta < 0.0:
    ftheta += twopi
  #print 'theta in degrees', math.degrees(ftheta)
  return ftheta

thetatestset = [
  (0.0, 0.0, 0.0),
  (0.0, wb, 90.0),
  (0.0, 2.0*wb, 180.0),
  (0.0, 3.0*wb, 270.0),
  (0.0, 4.0*wb, 0.0),
  (0.0, 5.0*wb, 90.0),

  (wb, 0.0, 270.0),
  (2.0*wb, 0.0, 180.0),
  (3.0*wb, 0.0, 90.0),
  (4.0*wb, 0.0, 0.0),
  (5.0*wb, 0.0, 270.0),

  (100.0, 100.0, 0.0),

  (wb, wb+math.sin(rad30)*wb, 30.0),
  (wb, wb-math.sin(rad30)*wb, 330.0),

  (0.0, -wb, 270.0),
  (-wb, 0.0, 90.0),
]

def testtheta():
  for tstpt in thetatestset:
    ftheta = theta(tstpt[0], tstpt[1])
    print 'odometer (%.3f, %.3f): theta exepct: %.3f  --> theta got: %.3f' % \
        (tstpt[0], tstpt[1], tstpt[2], math.degrees(ftheta))

