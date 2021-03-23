################################################################################
#
# Pt.py
#

""" Point Module

RxR or ZxZ point container class module.

Author: Robin D. Knight
Email:  robin.knight@roadnarrowsrobotics.com
URL:    http://www.roadnarrowsrobotics.com
Date:   2006.01.21

Copyright (C) 2006.  RoadNarrows LLC.
"""

#
# All Rights Reserved
#
# Permission is hereby granted, without written agreement and without
# license or royalty fees, to use, copy, modify, and distribute this
# software and its documentation for any purpose, provided that
# (1) The above copyright notice and the following two paragraphs
# appear in all copies of the source code and (2) redistributions
# including binaries reproduces these notices in the supporting
# documentation.   Substantial modifications to this software may be
# copyrighted by their authors and need not follow the licensing terms
# described here, provided that the new terms are clearly indicated in
# all files where they apply.
#
# IN NO EVENT SHALL THE AUTHOR, ROADNARROWS LLC, OR ANY MEMBERS/EMPLOYEES
# OF ROADNARROW LLC OR DISTRIBUTORS OF THIS SOFTWARE BE LIABLE TO ANY
# PARTY FOR DIRECT, INDIRECT, SPECIAL, INCIDENTAL, OR CONSEQUENTIAL
# DAMAGES ARISING OUT OF THE USE OF THIS SOFTWARE AND ITS DOCUMENTATION,
# EVEN IF THE AUTHORS OR ANY OF THE ABOVE PARTIES HAVE BEEN ADVISED OF
# THE POSSIBILITY OF SUCH DAMAGE.
#
# THE AUTHOR AND ROADNARROWS LLC SPECIFICALLY DISCLAIM ANY WARRANTIES,
# INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND
# FITNESS FOR A PARTICULAR PURPOSE. THE SOFTWARE PROVIDED HEREUNDER IS ON AN
# "AS IS" BASIS, AND THE AUTHORS AND DISTRIBUTORS HAVE NO OBLIGATION TO
# PROVIDE MAINTENANCE, SUPPORT, UPDATES, ENHANCEMENTS, OR MODIFICATIONS.
#
################################################################################

import math


#-------------------------------------------------------------------------------
# Global Data
#-------------------------------------------------------------------------------

# tuple indices
X     = 0
Y     = 1


#-------------------------------------------------------------------------------
# CLASS: pt
#-------------------------------------------------------------------------------
class pt:
  """ RxR and ZxZ Point Container Class

      There a two sets of valid operands for (binary) operators:
        vector (point) and scaler (int or float)

      Vectors are of types:
        class instance pt 
        2-tuple
        comma separated x, y  (where applicable)
  """
  def __init__(self, *vect):
    """ Initialize point instance.

        Parameters:
          *vect - vector operand(s). None ==> (0, 0)
    """
    self.x, self.y = self._cvtvector(vect)

  def __add__(self, vect):
    """ Point translation + operator. 
    
        p.__add__(vect)  <==>  p.trans(vect)

        Returns a translated copy of point p.
    """
    return self.trans(vect)

  def __eq__(self, point):
    """ Logical == operator. 
    
        p.__eq__(point)  <==>  p.mag()==point.mag()
    """
    q = pt(point)
    return self.mag() == q.mag()

  def __ge__(self, point):
    """ Logical >= operator. 
    
        p.__ge__(point)  <==>  p.mag()>=point.mag()
    """
    q = pt(point)
    return self.mag() >= q.mag()

  def __gt__(self, point):
    """ Logical > operator. 
    
        p.__gt__(point)  <==>  p.mag()>point.mag()
    """
    q = pt(point)
    return self.mag() > q.mag()

  def __iter__(self):
    """ Point iterator.
    
        p.__iter__()  <==>  p.tuple().__iter__()
    """
    return (self.x, self.y).__iter__()

  def __le__(self, point):
    """ Logical <= operator. 
    
        p.__le__(point)  <==>  p.mag()<=point.mag()
    """
    q = pt(point)
    return self.mag() <= q.mag()

  def __len__(self):
    """ Length operator.
    
        p.__len__()  <==>  len(p.tuple())
    """
    return 2

  def __lt__(self, point):
    """ Logical < operator. 
    
        p.__lt__(point)  <==>  p.mag()<point.mag()
    """
    q = pt(point)
    return self.mag() < q.mag()

  def __mul__(self, opands):
    """ Point multiplication * operator. 
    
        p.__rmul__(vector)  <==>  p.dot(vector)
        p.__rmul__(scalar)  <==>  p.scale(scalar)

        Returns a * operated copy of point p.
    """
    q = self._cvt(opands)
    if type(q) == tuple:
      return self.dot(q)
    else:
      return self.scale(q)

  def __ne__(self, point):
    """ Logical != operator. 
    
        p.__ne__(point)  <==>  p.mag()!=point.mag()
    """
    q = self._cvt(point)
    return self.mag() != q.mag()

  def __repr__(self):
    """ Represent operator. 
    
        p.__repr__()  <==>  repr(p.tuple())
    """
    return repr((self.x, self.y))

  def __radd__(self, vect):
    """ Point right translation + operator. 
    
        p.__radd__(vect)  <==>  vect.trans(p)

        Returns a translated copy of point p.
    """
    return self.trans(vect)

  def __rmul__(self, opands):
    """ Point right multiplication * operator. 
    
        p.__rmul__(vector)  <==>  vector.dot(p)
        p.__rmul__(scalar)  <==>  p.scale(scalar)

        Returns a * operated copy of point p.
    """
    q = self._cvt(opands)
    if type(q) == tuple:
      q = pt(q)
      return q.dot((self.x, self.y))
    else:
      return self.scale(q)

  def __rsub__(self, opands):
    """ Point right negative translation - operator. 
    
        p.__rsub__(vect)  <==>  vect.trans((-p.x, -p.y))

        Returns a translated copy of point p.
    """
    q = pt(opands)
    return q.trans(-self.x, -self.y)

  def __sub__(self, opands):
    """ Point negative translation - operator. 
    
        p.__sub__(vect)  <==>  p.trans((-vect.x, -vect.y))

        Returns a translated copy of point p.
    """
    q = self._cvtvector(opands)
    return self.trans(-q[X], -q[Y])

  def mag(self):
    """ Vector magnitude
    
        Returns Euclidean distance from origin (0, 0).
    """
    return math.sqrt(math.pow(self.x, 2) + math.pow(self.y, 2))

  def theta(self):
    """ Vector angle from positive x-axis. 
    
        Returns angle in radians [0.0, pi].
    """
    if self.x == 0 and self.y == 0:
      return 0.0
    return math.acos(self.x/self.mag())

  def issame(self, *point):
    """ Returns True if points are identical, False otherwise. """
    q = self._cvt(*point)
    return (self.x, self.y) == q

  def tuple(self):
    """ Returns point 2-tuple. """
    return self.x, self.y

  def rot(self, phi):
    """ Rotate the point ccw about the origin (0, 0) by phi radians.

        Return Value:
          Returns a rotated copy of point p.
    """
    return pt(self.x * math.cos(phi) - self.y * math.sin(phi),
              self.x * math.sin(phi) + self.y * math.cos(phi))

  def scale(self, *opands):
    """ Scale the point.

        vector: p.scale((xscale, yscale)) -> x*xscale, y*yscale
        scalar: p.scale(scale) -> x*scale, y*scale
    
        Return Value:
          Returns a scaled copy of point p.
    """
    scaler = self._cvt(*opands)
    if type(scaler) == tuple:
      return pt(self.x * scaler[X], self.y * scaler[Y])
    else:
      return pt(self.x * scaler, self.y * scaler)

  def trans(self, *opands):
    """ Translate the point.

        vector: scale((xoff, yoff)) -> x+xoff, y+yoff
    
        Return Value:
          Returns a translated copy of point p.
    """
    voff = self._cvtvector(*opands)
    return pt(self.x + voff[X], self.y + voff[Y])

  def dot(self, *vect):
    """ Take the dot product of the point with a vector.
          p . vect

        Return Value:
          Returns the dot product scaler.
    """
    q = self._cvtvector(*vect)
    return self.x * q[X] + self.y * q[Y]

  def cross(self, *vect):
    """ Take the cross product of the point with a vector.
          p x vect

        Return Value:
          Returns the cross product magnitude of the z-axis vector.
    """
    q = self._cvtvector(*vect)
    return self.x * q[Y] - self.y * q[X]

  def _cvtvector(self, *vect):
    """ Convert vector operand(s) into 2-tuple (x, y). """
    n = len(vect)
    if n > 2:
      raise TypeError('too many operands: %s' % (self.__name__, repr(vect)))
    elif n == 2:
      if type(vect[X]) != float and type(vect[X]) != int: 
        raise TypeError("value '%s' is not an 'int' or 'float' type" \
            % (repr(vect[X])))
      if type(vect[Y]) != float and type(vect[Y]) != int: 
        raise TypeError("value '%s' is not an 'int' or 'float' type" \
            % (repr(vect[Y])))
      return vect[X], vect[Y]
    elif n == 1:
      vop = vect[0]
      if type(vop) == tuple:
        n = len(vop)
        if n == 2:
          return self._cvtvector(vop[X], vop[Y])
        elif n == 1:
          return self._cvtvector(vop[0])
        else:
          raise TypeError("tuple '%s' is wrong size" % (repr(vop)))
      elif isinstance(vop, pt):
        return vop.x, vop.y
      else:
        raise TypeError("cannot convert '%s' into a 2-tuple vector" \
            % (repr(vop)))
    else:
      return 0, 0

  def _cvtscalar(self, *scalar):
    """ Convert scalar operand into scalar. """
    if len(scalar) != 1:
      raise TypeError('too many operands: %s' % (self.__name__, repr(scalar)))
    elif type(scalar[0]) != float and type(scalar[0]) != int: 
      raise TypeError("value '%s' is not an 'int' or 'float' type" \
            % (repr(scalar[0])))
    else:
      return scalar[0]

  def _cvt(self, *opands):
    """ Convert operands into either a scalar s or a 2-tuple (x, y). """
    try:
      s = self._cvtscalar(*opands)
      return s
    except TypeError:
      pass
    return self._cvtvector(*opands)


#-------------------------------------------------------------------------------
# Test Code
#-------------------------------------------------------------------------------

if __name__ == '__main__':
  def execdata(name, data):
    print(name)
    for s in data:
      print('  ', s, '-->', end='')
      try:
        exec(s)
      except TypeError as msg:
        print(msg)

  def tstpt():
    name = 'assignment'
    data = [
      'print(pt(5, 6))',
      'print(pt((6, 7)))',
      'print(pt(8))',
      'print(pt((9,)))',
      'p = 3, 5; print(p)',
      'p = (4, 6); print(p)',
      'p = pt(55, 66); print(pt(p))',
      'p = pt(-10, -100); q = pt(1,1); p = q; print(p)'
    ]
    execdata(name, data)

    name = 'addition'
    data = [
      'p = pt(1,2); print(p + 2)',
      'p = pt(1,2); print(p + (2,))',
      'p = pt(1,2); print(p + (2,3))',
      'p = pt(1,2); q=pt(-1, 55); print(p + q)',
      'p = pt(1,2); print((2,3)) + p',
      'p = pt(1,2); print(2 + p)',
      'p = pt(1,2); p += (1000, 1000); print(p)',
      'p = pt(1,2); q = pt(1000, 1000); p += q; print(p)',
    ]
    execdata(name, data)

    name = 'subtraction'
    data = [
      'p = pt(1,2); print(p - 2)',
      'p = pt(1,2); print(p - (2,))',
      'p = pt(1,2); print(p - (2,3))',
      'p = pt(1,2); q=pt(-1, 55); print(p - q)',
      'p = pt(1,2); print((2,3) - p)',
      'p = pt(1,2); print(2 - p)',
      'p = pt(1,2); p -= (1000, 1000); print(p)',
      'p = pt(1,2); q = pt(1000, 1000); p -= q; print(p)',
    ]
    execdata(name, data)

    name = 'dot product/scale'
    data = [
      'p = pt(1,2); print(p * 2)',
      'p = pt(1,2); print(p * (2,))',
      'p = pt(1,2); print(p * (2,3))',
      'p = pt(1,2); q=pt(-1, 55); print(p * q)',
      'p = pt(1,2); print((2,3) * p)',
      'p = pt(1,2); print(2 * p)',
      'p = pt(1,2); p *= (1000, 1000); print(p)',
      'p = pt(1,2); q = pt(1000, 1000); p *= q; print(p)',
    ]
    execdata(name, data)

    name = 'magnitude'
    data = [
      'print(pt(1,2).mag())',
      'print(pt(1,0).mag())',
      'print(pt(0,1).mag())',
      'print(pt(1,1).mag())',
    ]
    execdata(name, data)

    name = 'cross product'
    data = [
      'print(pt(1,0).cross(1, 0))',
      'print(pt(1,0).cross(0, 1))',
      'print(pt(0,1).cross(1, 0))',
      'print(pt(0,1).cross(0, 1))',
      'print(pt(23,-6).cross(8, -44))',
    ]
    execdata(name, data)

    name = 'rotate'
    data = [
      'p= pt(1,0).rot(math.pi); print(p, p.mag())',
      'p= pt(1,0).rot(-math.pi); print(p, p.mag())',
      'p= pt(1,0).rot(2.0 * math.pi); print(p, p.mag())',
      'p= pt(1,0).rot(math.pi/4.0); print(p, p.mag())',
      'p= pt(1,1).rot(math.pi/4.0); print(p, p.mag())',
      'p= pt(1,1).rot(-math.pi/4.0); print(p, p.mag())',
      'p= pt(1,1).rot(-math.pi/2.0); print(p, p.mag())',
      'p= pt(53,42).rot(3.0); print(p, p.mag())',
    ]
    execdata(name, data)

  #--
  def main():
    """ Types Test Main """
    tstpt()

  # run test
  main()
