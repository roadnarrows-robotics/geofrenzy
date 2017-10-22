#! /usr/bin/env python

################################################################################
# 
# File: subtend.py
#
# Usage:  latlongmeters.py
#
# Description:
# Test subtened calculations for 2D polygonal shapes.
#
################################################################################

import math
import numpy as np

import matplotlib.pyplot as plt
from   matplotlib.path import Path
import matplotlib.patches as Patches

# Notes:
#   Solid angle Omega is 2D angle that on object subtends (subTheta, subPhi).
#   Sum of internal angles of convex polygon: 180(n-2).
#

#-------------------------------------------------------------------------------
# Mathy Routines
#-------------------------------------------------------------------------------

Origin  = (0.0, 0,0)

PI      = math.pi
TAU     = 2.0 * math.pi

RADIUS  = 0
THETA   = 1
X       = 0
Y       = 1
MIN     = 0
MAX     = 1

#
# Shape container class.
#
class Shape:
  def __init__(self, xy=[], rt=[], closed=True):
    self.xy     = xy
    self.rt     = rt
    self.numpts = 0
    self.closed = closed

    if len(self.xy) > 0:
      self.numpts = len(self.xy)
      self.xy = fpn(self.xy)
      self.rt = xy2rt(self.xy)
    elif len(self.rt) > 0:
      self.numpts = len(self.rt)
      self.rt = fpn(self.rt)
      self.xy = rt2xy(self.rt)
    else:
      self.numpts = 0

  def __repr__(self):
    return \
        "  {\n" + \
        "    xy={0},\n".format(rnd(self.xy, 2)) + \
        "    rt={0},\n".format(rnd(degrees(self.rt), 2)) + \
        "    closed={0}\n".format(self.closed) + \
        "  }"

#
# Indirect Subtend class.
#
class ISubtend:
  def __init__(self, index = 0, omega = TAU, shape=None):
    self.index  = index       # index into shape vertices
    self.omega  = omega       # subtended ccw angle from vertex (radians)
    self.vertex = (0.0, 0.0)  # starting vertex in shape indexed by index
    if shape is not None:
      self.setVertex(shape)   # set vertex from shape via index

  def __repr__(self):
    return \
        "{" + \
        "index={0}, vertex={1}, omega={2}".format(self.index,
            rnd(degrees(self.vertex), 2),
            rnd(degrees(self.omega), 2)) + \
        "}"

  def setVertex(self, shape):
    if self.index >= 0 and self.index < shape.numpts:
      self.vertex = shape.rt[self.index]
      return True
    else:
      self.theta = (0.0, 0.0)
      return False


# a0   | a1   | a1-a0 | delta
# ---- | ---- | ----- | -----
#   30 |   60 |    30 |    30
#   60 |   30 |   -30 |   -30
#  -30 |   60 |    90 |    90
#   60 |  -30 |   -90 |   -90
#  -95 |  -30 |    65 |    65
#  -30 |  -95 |   -65 |   -65
#  170 | -150 |  -320 |   -40
# -150 |  170 |   320 |    40
#  180 |    0 |  -180 |  -180
#    0 |  180 |   180 |   180
def delta(a0, a1):
  d = a1 - a0
  if d > 180:       # crossing -x axis from -y
    d = 360 - d
  elif d <= -180:   # crossing -x axis from +y
    d = -360 - d 
  return d

isfpn   = lambda x: type(x) is float or type(x) is int
ispolar = lambda x: type(x) is tuple and len(x) == 2


#
# Force a number or numbers to be floating-point numbers.
#
# obj       (Compound) object.
#
# Return fpn-ized object.
#
def fpn(obj):
  if isfpn(obj):  # scalar
    return float(obj)
  elif type(obj) is list:
    l = []
    for item in obj:
      l.append(fpn(item))
    return l
  elif type(obj) is tuple:
    l = []
    for item in obj:
      l.append(fpn(item))
    return tuple(l)
  else:
    return obj
#
# Convert geometric object in degrees to equivalent in radians.
#
# obj   Geomentric (compound) object specified in degrees.
#
# Return Object in radians.
#
def radians(obj):
  if isfpn(obj):              # scalar
    return math.radians(obj)
  elif ispolar(obj):          # (assumed) polar point
    return (obj[RADIUS], math.radians(obj[THETA]))
  elif type(obj) is list:     # list of degrees and/or polar tuples
    l = []
    for item in obj:
      l.append(radians(item))
    return l
  else:
    return obj

#
# Convert geometric object in radians to equivalent in degrees.
#
# obj   Geomentric (compound) object specified in radians.
#
# Return object in degrees.
#
def degrees(obj):
  if isfpn(obj):              # scalar
    return math.degrees(obj)
  elif ispolar(obj):          # (assumed) polar point
    return (obj[RADIUS], math.degrees(obj[THETA]))
  elif type(obj) is list:     # list of degrees and/or polar tuples
    l = []
    for item in obj:
      l.append(degrees(item))
    return l
  else:
    return obj

#
# Take the supplementary angle(s) of geometric object in radians.
#
# obj   Geomentric (compound) object specified in radians.
#
# Return object of supplementary angles in radians.
#
def supplementary(obj):
  if isfpn(obj):              # scalar
    if obj >= 0.0:
      return math.pi - obj
    else:
      return -math.pi - obj
  elif ispolar(obj):          # (assumed) polar point
    return (obj[RADIUS], supplementary(obj[THETA]))
  elif type(obj) is list:     # list of degrees and/or polar tuples
    l = []
    for item in obj:
      l.append(supplementary(item))
    return l
  else:
    return obj

#
# Round object to given precision.
#
# obj       (Compound) object.
# ndigits   Number of digits of precision.
#
# Return rounded object.
#
def rnd(obj, ndigits):
  if isfpn(obj):  # scalar
    return round(obj, ndigits)
  elif type(obj) is list:
    l = []
    for item in obj:
      l.append(rnd(item, ndigits))
    return l
  elif type(obj) is tuple:
    l = []
    for item in obj:
      l.append(rnd(item, ndigits))
    return tuple(l)
  else:
    return obj

#
# Keep angle a (radian) in range (-pi, pi].
#
# obj   Geomentric (compound) object specified in radians.
#
# Returns object with angles in (-pi, pi].
#
def pi2pi(obj):
  if isfpn(obj):              # scalar
    if obj <= math.pi and obj > -math.pi:
      return obj
    elif obj > math.pi:
      return obj - 2 * math.pi
    elif obj <= -math.pi:
      return 2 * math.pi + obj
  elif ispolar(obj):          # (assumed) polar point
    return (obj[RADIUS], pi2pi(obj[THETA]))
  elif type(obj) is list:     # list of degrees and/or polar tuples
    l = []
    for item in obj:
      l.append(pi2pi(item))
    return l
  else:
    return obj

#
# Calculate Euclidean distant from pt0 to pt1.
#
def L2(pt0, pt1):
  return math.sqrt(pow(pt1[X]-pt0[X], 2) + pow(pt1[Y]-pt0[Y], 2))

#
# Convert point(s) in polar coordinates to cartesian coordinate equivalents.
#
# rt   Point or points in polar coordinates (r,theta).
#
# Return Cartesian point(s).
#
def rt2xy(rt):
  if type(rt) is tuple:
    return (rt[RADIUS] * math.cos(rt[THETA]), rt[RADIUS] * math.sin(rt[THETA]))
  elif type(rt) is list:     # list polar tuples
    xy = []
    for pt in rt:
      x = pt[RADIUS] * math.cos(pt[THETA])
      y = pt[RADIUS] * math.sin(pt[THETA])
      xy += [(x, y)]
    return xy
  else:
    return rt

#
# Convert point(s) in cartesian coordinates to polar coordinate equivalents.
#
# xy  Point or points in cartesian coordinates (x,y).
#
# Return Polar point(s).
#
def xy2rt(xy):
  if type(xy) is tuple:
    return (L2(Origin, xy), math.atan2(xy[Y], xy[X]))
  elif type(xy) is list:     # list of cartesian tuples
    rt = []
    for pt in xy:
      r = L2(Origin, pt)
      t = math.atan2(pt[Y], pt[X])
      rt += [(r, t)]
    return rt
  else:
    return xy

#
# Find bounding box around polygonal shape.
#
# The sides of the bounding box are parallel to axes.
#
# shape  Polygonal shape in cartesian coordinates (x,y).
#
# Return [(minX, maxX), (minY, maxY)]
#
def bounding(shape):
  minX = maxX = shape[0][X]    # seed min,max x
  minY = maxY = shape[0][Y]    # seed min,max y
  for pt in shape[1:]:
    if pt[X] < minX:
      minX = pt[X]
    elif pt[X] > maxX:
      maxX = pt[X]
    if pt[Y] < minY:
      minY = pt[Y]
    elif pt[Y] > maxY:
      maxY = pt[Y]
  return [(minX, maxX), (minY, maxY)]

#
# Test if point inside closed, simple polygon.
#
# pt      Point in cartesian coordinates.
# polygon Polygon as a list of points in cartesian coordinates.
#
# Returns True or False.
#
def pipCn(pt, poly):
  n  = len(poly)
  cn = 0;

  # minimum polygon is a triangle.
  if n < 3:
    return False
 
  #
  # Count the edge crossings.
  #
  for i in range(n):

    j = (i + 1) % n

    # v0-v1 edge
    v0 = poly[i];
    v1 = poly[j];

    #
    # Upward crossing excludes endpoint. Downward crossing excludes
    # startpoint.
    #
    if (v0[Y] <= pt[Y]) and (v1[Y] >  pt[Y]) or \
       (v0[Y] >  pt[Y]) and (v1[Y] <= pt[Y]):

      # slope - no divide by zero exception given the above test   
      vt = (pt[Y]  - v0[Y]) / (v1[Y] - v0[Y]);

      # compute the actual edge-ray intersect x-coordinate
      if  pt[X] <  v0[X] + vt * (v1[X] - v0[X]):
        cn += 1   # a valid crossing of y=pt[Y] right of pt.x()

  if cn & 0x01:
    return True
  else:
    return False

#
# Test if shape crosses the -x axis.
#
# shape Polygonal shape as a list of points in cartesian coordinates.
#
# Returns True or False.
#
def crossesNegXAxis(shape):
  n = len(shape)

  for i in range(n):
    j = (i + 1) % n

    # edge
    vi = shape[i];
    vj = shape[j];

    # near vertical slope
    if math.fabs(vj[X] - vi[X]) < 0.001:
      # negative x's
      if vi[X] < 0.0 and vj[X] < 0.0:
        if vi[Y] <= 0.0 and vj[Y] >= 0.0:   # crosses -x axis from QIII to QII
          return True
        elif vi[Y] >= 0.0 and vj[Y] <= 0.0: # crosses -x axis from QII to QIII
          return True
    else:
      m = (vj[Y] - vi[Y]) / (vj[X] - vi[X])   # slope
      b = vi[Y] - m * vi[X]                   # intercept
      if math.fabs(m) > 0.001:
        x = -b / m                            # solve for x at y=0
        if x < 0.0:
          if x >= vi[X] and x <= vj[X]:
            return True
          if x >= vj[X] and x <= vi[X]:
            return True

  return False

#
# Find minimum and maximum theta.
#
# shape  Polygonal shape in polar coordinates (r,theta).
#
# Return (minTheta, maxTheta)
#
def minmaxTheta(shape):
  m = shape[0][THETA]    # seed minimum theta
  M = shape[0][THETA]    # seed maximum theta
  for pt in shape[1:]:
    if pt[THETA] < m:
      m = pt[THETA]
    elif pt[THETA] > M:
      M = pt[THETA]
  return (m, M)

#
# Find minimum and maximum radii.
#
# shape  Polygonal shape in polar coordinates (r,theta).
#
# Return (minRadius, maxRadius)
#
def minmaxRadius(shape):
  m = shape[0][RADIUS]    # seed minimum radius
  M = shape[0][RADIUS]    # seed maximum radius
  for pt in shape[1:]:
    if pt[RADIUS] < m:
      m = pt[RADIUS]
    elif pt[RADIUS] > M:
      M = pt[RADIUS]
  return (m, M)

#
# Determine if rotation from angle a0 to a1 is ccw.
#
# This version is ugly as sin.
#
# a0    Starting angle (degrees).
# a1    Ending angle (degrees).
#
# Return True if ccw, False otherwise.
#
def ccw(a0, a1):
  d = a1 - a0
  if a0 >= 0.0 and a1 >= 0.0:
    if d >= 0.0:
      return True
    else:
      return False;
  elif a0 >= 0.0 and a1 < 0.0:
    if d >= -180.0:
      return False
    else:
      return True
  elif a0 < 0.0 and a1 >= 0.0:
    if d <= 180.0:
      return True
    else:
      return False
  elif a0 < 0.0 and a1 < 0.0:
    if d >= 0.0:
      return True
    else:
      return False

#
# Calculate specified polygon rotation order.
#
# poly  Polygon in polar coordinates (r,theta).
#
# Returns winding >= 0 if ccw, winding < 0 if cw.
#
def rotRT(poly):
  wind = 0
  for i in range(len(poly)):
    j = (i + 1) % len(poly)
    if ccw(poly[i][THETA], poly[j][THETA]):
      wind += 1
    else:
      wind += -1
  return wind

#
# Calculate bounding box angles from origin.
#
# bbox  Bound box.
#
# Returns list of thetas.
#
def bboxThetas(bbox):
  a0 = math.degrees(math.atan2(bbox[Y][MIN], bbox[X][MIN]))   # min y, min x
  a1 = math.degrees(math.atan2(bbox[Y][MAX], bbox[X][MIN]))   # max y, min x
  a2 = math.degrees(math.atan2(bbox[Y][MIN], bbox[X][MAX]))   # min y, max x
  a3 = math.degrees(math.atan2(bbox[Y][MAX], bbox[X][MAX]))   # max y, max x
  return [a0, a1, a2, a3]

#
# Calculate the subtended angle of the shape.
#
def subtendShape(shape):
  if pipCn(Origin, shape.xy):
    index = -1
    omega = 2 * math.pi
    return ISubtend(index, omega)

  theta   = float(shape.rt[0][THETA])
  theta_  = supplementary(theta)
  #print 'DBG: {0:>6.1f} {1:>6.1f}'.format(degrees(theta), degrees(theta_))

  i = j = 0
  m = theta     # seed minimum theta
  M = theta     # seed maximum theta

  i_ = j_ = 0
  m_ = theta_   # seed minimum supplementary theta
  M_ = theta_   # seed maximum supplementary theta

  k = 1

  for pt in shape.rt[k:]:
    theta   = float(pt[THETA])
    theta_  = supplementary(theta)
    #print 'DBG: {0:>6.1f} {1:>6.1f}'.format(degrees(theta), degrees(theta_))
    if theta < m:
      i = k
      m = theta
    elif theta > M:
      j = k
      M = theta
    if theta_ < m_:
      i_ = k
      m_ = theta_
    elif theta_ > M_:
      j_ = k
      M_ = theta_

    k += 1

  if crossesNegXAxis(shape.xy):
    #print 'DBG: does cross -x'
    index = j_
    omega = math.fabs(M_ - m_)
  else:
    #print 'DBG: does not cross -x'
    index = i
    omega = math.fabs(M - m)

  return ISubtend(index, omega, shape)


#-------------------------------------------------------------------------------
# Graphics Routines
#-------------------------------------------------------------------------------

def drawPolygon(fig, poly):
  verts = poly + [poly[0]]
  codes = [Path.MOVETO]
  codes += [Path.LINETO] * (len(verts) - 2)
  codes += [Path.CLOSEPOLY]
  pth = Path(verts, codes)
  ax = fig.add_subplot(111)
  patch = Patches.PathPatch(pth, facecolor='orange', lw=2)
  ax.add_patch(patch)
  return ax

def drawMultiLine(fig, poly):
  verts = poly
  codes = [Path.MOVETO]
  codes += [Path.LINETO] * (len(verts) - 1)
  ax = fig.add_subplot(111)
  for j in range(1, len(verts)):
    line
    line(verts[j-1], verts[j], color='orange', linewidth=3)
  return ax

def drawBBox(fig, bbox):
  verts = []
  verts.append((bbox[X][MIN], bbox[Y][MIN]))  # min x, min y
  verts.append((bbox[X][MAX], bbox[Y][MIN]))  # max x, min y
  verts.append((bbox[X][MAX], bbox[Y][MAX]))  # max x, max y
  verts.append((bbox[X][MIN], bbox[Y][MAX]))  # min x, max y
  verts.append(verts[0])                      # close
  codes = [Path.MOVETO]
  codes += [Path.LINETO] * (len(verts) - 2)
  codes += [Path.CLOSEPOLY]
  pth = Path(verts, codes)
  bx = fig.add_subplot(111)
  patch = Patches.PathPatch(pth, facecolor='None', lw=1)
  bx.add_patch(patch)
  return bx

def drawActualSubtend(fig, subtend, r):
  drawSubtend(fig, subtend, r, 'a', color='green', linestyle='solid')
  drawAngleCallout(fig, subtend.vertex[THETA], subtend.omega, r)

def drawCalcSubtend(fig, subtend, r):
  drawSubtend(fig, subtend, r, 'c', color='red', linestyle='dashdot')

def drawSubtend(fig, subtend, r, subscript, **kwargs):
  # default color
  if not kwargs.has_key('color'):
    kwargs['color'] = 'black'

  # starting vertex polar coordinates
  rt = subtend.vertex

  # start, span in degrees
  theta = degrees(rt[THETA])
  omega = degrees(subtend.omega)
  if omega >= 360.0:
    theta = 0.0
    omega = 360.0

  label = r'$\theta_{0} = ${1}  $\omega_{0} = ${2}'.format(
      subscript, rnd(theta, 1), rnd(omega, 1))

  # Origin is not within the shape
  if subtend.omega < TAU:
    # starting vertex
    xy = rt2xy(rt)
    scatter([xy], color=kwargs['color'], linewidth=3)

    # start line
    rt0 = (r, rt[THETA])
    xy0 = rt2xy(rt0)
    line(Origin, xy0, linewidth=2, label=label, **kwargs)

    # rotate ccw subtended degrees
    rt1 = xy2rt(xy0)
    rt1 = (r, pi2pi(rt1[THETA] + subtend.omega))
    xy1 = rt2xy(rt1)
    line(Origin, xy1, linewidth=2, **kwargs)

  # 360 subtended angle - draw only the origin
  else:
    # origin
    scatter([Origin], color=kwargs['color'], linewidth=3)
    an = np.linspace(0, 2 * np.pi, 50)
    r = 0.5
    plt.plot(r * np.cos(an), r * np.sin(an), label=label, **kwargs)

def drawAngleCallout(fig, theta, omega, r):
  if omega < 0.01 or omega >= TAU:
    return

  if r <= 1.0:
    r = r * 0.9
  else:
    r = r * 0.2

  an = np.linspace(theta, theta+omega, 50)
  plt.plot(r * np.cos(an), r * np.sin(an), linewidth=1, color='black')

  x0 = r * np.cos(an[-1])
  y0 = r * np.sin(an[-1])
  x1 = r * np.cos(an[-2])
  y1 = r * np.sin(an[-2])
  dx = x1 - x0
  dy = y1 - y0
  hs = L2(Origin, (dx, dy))
  if hs < 0.2:
    s = 0.2 / hs
    dx *= s
    dy *= s
  
  verts = [ (x0, y0),
            #(x0 + dx, y0 + dy),
            (x0 + dx - dy * 0.5, y0 + dy + dx * 0.5),
            (x0 + dx + dy * 0.5, y0 + dy - dx * 0.5)
          ]
  verts.append(verts[0])                      # close

  codes  = [Path.MOVETO]
  codes += [Path.LINETO] * (len(verts) - 2)
  codes += [Path.CLOSEPOLY]
  
  pth = Path(verts, codes)
  patch = Patches.PathPatch(pth, facecolor='black', lw=1)

  ax = fig.add_subplot(111)
  ax.add_patch(patch)

  #plt.arrow(x0, y0, dx, dy, color='blue', fill=True,
  #    width=0.0,        # arrow tail width
  #    head_width=hs,    # arrow head width
  #    head_length=hs)   # arrow head length

def line(pt0, pt1, **kwargs):
  x = [pt0[0], pt1[0]]
  y = [pt0[1], pt1[1]]
  plt.plot(x, y, **kwargs)

def scatter(pts, **kwargs):
  x = []
  y = []
  for pt in pts:
    x.append(pt[X])
    y.append(pt[Y])
  plt.scatter(x, y, **kwargs)
  
def xaxis(ax, minmax):
  m = math.fabs(minmax[0])
  M = math.fabs(minmax[1])
  if M >= m:
    x = round(M + 1, 0)
  else:
    x = round(m + 1, 0)
  ax.set_xlim(-x, x)

def yaxis(ax, minmax):
  m = math.fabs(minmax[0])
  M = math.fabs(minmax[1])
  if M >= m:
    y = round(M + 1, 0)
  else:
    y = round(m + 1, 0)
  ax.set_ylim(-y, y)

def xyaxis(ax, bbox):
  r = [math.fabs(bbox[0][0]), math.fabs(bbox[0][1]),
       math.fabs(bbox[1][0]), math.fabs(bbox[1][1])]
  m = r[0]
  for v in r[1:]:
    if v > m:
      m = v
  x = round(m + 2, 0)
  y = x
  ax.axis('equal')
  #ax.axis([-x, x, -y, y])
  #ax.set_xlim(-x, x)
  #ax.set_ylim(-y, y)

def drawOrigin():
  s = 0.25
  line((-s, 0.0), (s, 0.0), color='black', linewidth=3)
  line((0.0, -s), (0.0, s), color='black', linewidth=3)


#-------------------------------------------------------------------------------
# Main
#-------------------------------------------------------------------------------

#if __name__ = '__main__':
if True:

  class testcase:
    def __init__(self, desc, shape, subtend_a):
      self.desc = desc
      self.shape = shape
      self.subtend_a = subtend_a

  # .  .  .  .  .  .  .  .  .  .  .  .  .  .  .  .  .  .  .  .  .  .  .  .  .  .
  # Sets of Test Cases.
  # 
  # Each test case specifies:
  # 'test'    Short decription
  # 'shape'   Geometric 2D shape specified in polar radius,theta coordinates
  #           where theta is in degrees. The last ... RDK
  # 'actual'  Ground truth of the acutal indirect subtended angle i omega
  #           i is the index into the shape and omega is the ccw subtended
  #           angle from the point at i specified in degrees.
  # .  .  .  .  .  .  .  .  .  .  .  .  .  .  .  .  .  .  .  .  .  .  .  .  .  .

  #
  # Triangle polygon test set.
  #
  TriTestSet = [
      { 'test':   'triangle - ccw, centered on the -x axis',
        'shape':  Shape(rt=radians([(2, 150), (4, 180), (2, -150)])),
        'actual': ISubtend(0, radians(60))
      },
      { 'test': 'triangle - cw, centered on the -x axis',
        'shape': Shape(rt=radians([(2, -150), (4, 180), (2, 150)])),
        'actual': ISubtend(2, radians(60))
      },
      { 'test': 'triangle - ccw, origin within',
        'shape': Shape(rt=radians([(10, -95), (1, 1), (8, 140)])),
        'actual': ISubtend(-1, radians(360))
      },
      { 'test': 'triangle - cw, centered on the +x axis',
        'shape': Shape(rt=radians([(2, 30), (4, 0), (2, -30)])),
        'actual': ISubtend(2, radians(60))
      },
      { 'test': 'triangle - ccw, centered the on the +x axis',
        'shape': Shape(rt=radians([(2, -30), (4, 0), (2, 30)])),
        'actual': ISubtend(0, radians(60))
      },
      { 'test': 'triangle - ccw, QII, tip on the -x axis',
        'shape': Shape(rt=radians([(4, 180), (2, 110), (5, 140)])),
        'actual': ISubtend(1, radians(70))
      },
      { 'test': 'triangle - cw, QIII, tip on the -x axis',
        'shape': Shape(rt=radians([(4, 180), (2, -110), (5, -140)])),
        'actual': ISubtend(0, radians(70))
      },
      { 'test': 'triangle - ccw, centered the on the +y axis',
        'shape': Shape(rt=radians([(2, 90), (4, 45), (4, 135)])),
        'actual': ISubtend(1, radians(90))
      },
      { 'test': 'triangle - cw, centered the on the -y axis',
        'shape': Shape(rt=radians([(2, -90), (4, -135), (4, -45)])),
        'actual': ISubtend(1, radians(90))
      },
  ]

  #
  # Quadrilateral polygon test set.
  #
  QuadTestSet = [
      { 'test':   'square - ccw, origin within',
        'shape':  Shape(rt=radians([(2, 45), (2, 135), (2, -135), (2, -45)])),
        'actual': ISubtend(-1, radians(360))
      },
      { 'test':   'quadrilateral - cw, centered on the -x axis',
        'shape':  Shape(rt=radians([(4, 140), (4, -140), (2, -110), (2, 110)])),
        'actual': ISubtend(3, radians(140))
      },
      { 'test':   'quadrilateral - cw, centered on the -x axis, back visable',
        'shape':  Shape(rt=radians([(8, 120), (8, -120), (2, -130), (2, 130)])),
        'actual': ISubtend(0, radians(120))
      },
  ]

  #
  # General polygon test set.
  #
  PolyTestSet = [
    { 'test':   'polygon - ccw, 6-gon, concave, QI-IV-I-II',
      'shape':  Shape(rt=radians([(1, 30), (3, -20), (6, 60), (5, 65),
                                  (9, 80), (6, 120)])),
      'actual': ISubtend(1, radians(140))
    },
    { 'test':   'polygon - ccw, 6-gon, concave, 6 origin within',
      'shape':  Shape(rt=radians([(3, 30), (5, 90), (2, 135), (4, 180),
                                  (5, -90), (2, -25)])),
      'actual': ISubtend(-1, radians(360))
    },
    { 'test':   'polygon - ccw, 6-gon, concave, wrapped around origin',
      'shape':  Shape(rt=radians([(3, 50), (5, 135), (2, 15), (5, -135),
                                  (3, -90), (6, 25)])),
      'actual': ISubtend(3, radians(270))
    },
    { 'test':   'polygon - ccw, 6-gon, concave, wrapped around origin',
      'shape':  Shape(rt=radians([(3, 130), (5, 45), (2, 165), (5, -45),
                                  (3, -90), (6, 155)])),
      'actual': ISubtend(1, radians(270))
    },
    { 'test':   'polygon - ccw, 10-gon, concave, star, QI-IV',
      'shape':  Shape(xy=[(0, 0), (2.2, 0), (3, 2), (3.8, 0), (6, 0),
                          (4.2, -1.0), (5, -4), (3, -2), (1, -4), (2, -1)]),
      'actual': ISubtend(8, radians(109.7))
    },
  ]

  #
  # Open polygonals test set.
  #
  WallTestSet = [
    { 'test':   'wall - accros the +x axis',
      'shape':  Shape(rt=radians([(3, -30), (4, 30)]), closed=False),
      'actual': ISubtend(0, radians(60))
    },
    { 'test':   'wall - accros the +y axis',
      'shape':  Shape(rt=radians([(3, 30), (5, 130)]), closed=False),
      'actual': ISubtend(0, radians(100))
    },
    { 'test':   'wall - accros the -x axis',
      'shape':  Shape(rt=radians([(3, -120), (1, 170)]), closed=False),
      'actual': ISubtend(1, radians(70))
    },
    { 'test':   'wall - accros the -y axis',
      'shape':  Shape(rt=radians([(3, -30), (6, -135)]), closed=False),
      'actual': ISubtend(1, radians(105))
    },
    { 'test':   'wall - connected accros the QI-QIV',
      'shape':  Shape(rt=radians([(3, 30), (6, -45), (4, -90)]), closed=False),
      'actual': ISubtend(2, radians(120))
    },
  ]

  # All test cases
  AllTestCases  = TriTestSet + QuadTestSet + PolyTestSet + WallTestSet

  # Graphica on/off
  Graphics = True

  #
  # Run a test.
  #
  def runtest(n):
    # test case
    tc        = AllTestCases[n]       # the test case 
    desc      = tc['test']            # test descriptive title
    shape     = tc['shape']           # shape polar coordinates (radius,degrees)
    subtend_a = tc['actual']          # actual ccw point and subtended angle

    subtend_a.setVertex(shape)        # assign shape vertex to subtended

    # calculations
    bbox      = bounding(shape.xy)    # bounding box
    bboxT     = bboxThetas(bbox)      # angles subtended by bounding box
    mMT       = minmaxTheta(shape.rt) # min,max shape thetas
    mMR       = minmaxRadius(shape.rt) # min,max shape thetas
    rot       = rotRT(shape.rt)       # ccw or cw rotataion
    subtend_c = subtendShape(shape)   # shape subtended ccw (i, subtended)
  
    title = 'Test {0}: {1}'.format(n, desc)

    # print results
    print title
    print '  shape:\n',         shape
    print '  bbox:           ', rnd(bbox, 2)
    print '  thetas(bbox):   ', rnd(bboxT, 2)
    print '  minmax theta:   ', ( rnd(degrees(mMT[MIN]),2),
                                  rnd(degrees(mMT[MAX]),2) )
    print '  minmax radius:  ', rnd(mMR, 2)
    print '  order:          ', rot
    print '  subtend(actual):', subtend_a
    print '  subtend(result):', subtend_c
  
    # graph results
    if Graphics:
      fig = plt.figure()

      if shape.closed:
        ax = drawPolygon(fig, shape.xy)
      else:
        ax = drawMultiLine(fig, shape.xy)

      #xaxis(ax, bbox[0])
      #yaxis(ax, bbox[1])
      xyaxis(ax, bbox)
      ax.grid(True)
      drawOrigin()

      drawBBox(fig, bbox)

      drawActualSubtend(fig, subtend_a, mMR[MAX])
      drawCalcSubtend(fig, subtend_c, mMR[MAX])

      plt.suptitle(title, fontsize=16)
      plt.legend(loc='best', ncol=1)

      plt.show()
  
  #
  # Main Loop
  #
  print " Polygonal Shapes Testing"
  print "('h' for help, 'q' to quit)"
  print

  quit = False

  while not quit:
    cmd = raw_input("cmd> ")

    if len(cmd) == 0:
      continue
    
    if cmd == 'h':
      print "'a' - Run all test cases."
      print "'h' - Print this help."
      print "'l' - List available test cases."
      print "'q' - Quit."
      print "<n> - Run test case n."
    elif cmd == 'q':
      quit = True
    elif cmd == 'l':
      print "Test Cases:"
      for n in range(len(AllTestCases)):
        print "  {0:2}: {1}.".format(n, AllTestCases[n]['test'])
    elif cmd == 'a':
      for n in range(len(AllTestCases)):
        runtest(n)
    else:
      try:
        n = int(cmd)
      except ValueError:
        print "Error: Command '{0}' is unknown.".format(cmd)
        continue
      if n < 0 or n > len(AllTestCases):
        print "Error: Test {0} does not exist.".format(n)
        continue
      else:
        runtest(n)
