#! /usr/bin/env python

################################################################################
# 
# File: latlongmeters.py
#
# Usage:  latlongmeters.py [OPTIONS] [shapesfile [fcfile]]
#         latlongmeters.py --help
#
# Description:
#  Maps a Json shapes data, with shapes specified in meters, to geographic
#  latitude and longitude coordinates.
#
# See Also:
#   The shape files triangle.json, ..., tee.json serve as examples.
#
################################################################################

import sys
import os
import getopt
import json
import math as m
import numpy as np
import pprint

# ------------------------------------------------------------------------------
# The Help
# ------------------------------------------------------------------------------

def printHelp():
  print """\
Name:  latlongmeters.py - Map shapes to geographic coordinates.

Usage: latlongmeters.py [OPTIONS] [SHAPESFILE [FCFILE]]
       latlongmeters.py --help

Description:
  The latlongmeters.py utility maps Json shapes data into geographic
  latitude and longitude coordinates. The output is in the format of a
  Geofrenzy feature collection Json file. 

  If a Json shapes file SHAPESFILE is specified, the geographic mapping is
  auto-generated without any user interaction. Otherwise, interactive mode is
  entered to gather the shape data from the user.

  If a Json feature collection file FCFILE is specified, the geographic mapped
  data replaces the specified feature coordinates data. Otherwise a default
  feature collection template is used.

  If an output file is specified, the output overwrites that file. Otherwise
  the output is written to stdout.

  List of built-in geographic positions:
  {0}

Options:
  Mandatory  arguments  to  long  options are mandatory for short options too.
  -f, --feature=N     Update feature (index) N in the feature collection.
                      Default: 0
  -o, --output=FILE   Ouput feature collection with the mapped shapes to
                      FILE. The FILE will be overwritten. If no output option
                      is specfied, output will be written to stdout.
  -t, --trace         Trace the mapping.
  -h, --help          Print this help.

Json Shapes File:
  # Json Name-Values
  "starting_longitude": LON | BUILTIN # (required)
  "starting_latitude":  LAT | BUILTIN # (required)
  "scale":              SCALE         # scale shapes (default: 1.0)
  "rotate":             DEG           # rotate shapes degrees ccw (default: 0.0)
  "translate":          [X, Y]        # translate shapes (default: [0.0, 0.0] 
  "paths":              [path, path, ...]
  "path":               [[X, Y], [X, Y], ...]

  # Values
  LAT     ::= [-90.0, 90.0]
  LON     ::= [-180.0, 180.0]
  BUILTIN ::= string
  DEG     ::= [-360.0, 360.0]
  SCALE   ::= > 0.0
  X,Y     ::= meters, meters

See Also:
  The shape files hexagon.json, rectangle.json, tee.json and triangle.json
  serve as examples.
""".format(BuiltInGeoCoords.keys())

Argv0   = os.path.basename(__file__)    # command short name

EC_OK   = 0   # ok exit code
EC_ARGS = 2   # command line interface error exit code
EC_EXEC = 4   # execution error exit code

##
# \brief Print optional (error) message and exit program.
#
# \param  ec    Exit code
# \param  msg   Exit message.
#
def adios(ec, msg = None):
  if msg:
    if ec == EC_OK:
      print msg
    else:
      print >>sys.stderr, "{0}: Error: {1}".format(Argv0, msg)
  sys.exit(ec)


# ------------------------------------------------------------------------------
# Json Data and Routines
# ------------------------------------------------------------------------------

#
# Some cool built-in geographic coordinates in degress latitude and longitude.
#
BuiltInGeoCoords = {
  #'AVC2017':      (0.0, 0.0),  # RDK TODO need real Denver location coordinates
  'AVC2017':      (40.38085964, -105.099197), # RDK Use Boulder for now
  'Berlin':       (52.5200, 13.4050),
  'Boulder':      (40.38085964, -105.099197),
  'Brisbane':     (-27.4698, 153.0251),
  'Fiji':         (-17.7134, 178.0650),
  'Guam':         (13.4443, 144.7937),
  'New York':     (40.7128, -74.0059),
  'Las Vegas':    (36.1699, -115.1398),
  'Los Angeles':  (34.0522, -118.2437)
}

##
# Default input shapes Json dictionary.
#
InDft = {
  "starting_longitude": 0.0,
  "starting_latitude":  0.0,
  "scale":              1.0,
  "translate":          [0.0, 0.0],
  "rotate":             0.0,
  "paths":              []
}

##
# Default output feature collection Json dictionary.
#
OutDft = {
  "type": "FeatureCollection",
  "features":
  [
    {
      "type": "Feature",
      "properties":
      {
        "class_metadata":
        {
          "class_idx": 1,
          "entitlements":
          [
            {
              "ent_idx":      205,
              "ent_base":     "color",
              "color_red":    100,
              "color_green":  100,
              "color_blue":   150,
              "color_alpha":  255
            }
          ]
        },
        "inout":"o"
      },
      "geometry":
      {
        "type": "Polygon",
        "coordinates": []
      }
    }
  ]
}

##
# \brief Get feature <n> coordinates data.
#
# \param fc   Feature collection dictionary.
# \param n    Feature index.
#
# \return Feature geometry coordinates data lists.
#
def getFeatCoord(fc, n):
  try:
    return fc['features'][n]['geometry']['coordinates']
  except KeyError, inst:
    adios(EC_EXEC, "FeatureCollection does not have key {0}.".format(inst))
  except IndexError, inst:
    adios(EC_EXEC, "FeatureCollection feature {0} does not exist.".format(n))

##
# \brief Set feature <n> coordinates data.
#
# \param fc     Feature collection dictionary.
# \param n      Feature index.
# \param coord  Feature geometry coordinates data lists.
#
def setFeatCoord(fc, n, coord):
  try:
    fc['features'][n]['geometry']['coordinates'] = coord
  except KeyError, inst:
    adios(EC_EXEC, "FeatureCollection does not have key {0}.".format(inst))
  except IndexError, inst:
    adios(EC_EXEC, "FeatureCollection feature {0} does not exist.".format(n))

##
# \brief Force ASCII encoding when parsing Json files.
#
def ascii_encode_dict(data):
  ascii_encode = lambda x: x.encode('ascii') if isinstance(x, unicode) else x 
  return dict(map(ascii_encode, pair) for pair in data.items())

##
# \brief Read and parse Json file into a dictionary.
#
# \param filename   Json file name.
#
# \return Parsed dictionary.
#
def readJson(filename):
  try:
    fp = open(filename)
  except IOError:
    adios(EC_ARGS, "Cannot open file {0}".format(filename))
  try:
    d = json.load(fp, object_hook=ascii_encode_dict)
  except ValueError, inst:
    fp.close()
    adios(EC_EXEC, "{0}.".format(inst))
  fp.close()
  return d


# ------------------------------------------------------------------------------
# Earthy Conversions
# ------------------------------------------------------------------------------

# Radius of earth in KM
Re = 6378.137

##
# degrees --> radians
def radians(deg):
  return m.pi * deg / 180.0

##
# radians --> degrees
def degrees(rad):
  return 180.0 * rad / m.pi

##
# \brief Measure distance between two latitue,longitude positions.
#
# Simplified "Earth is a ball" measurement.
#
# \param lat1   Source geographic latitude (degrees)
# \param lon1   Source geographic longitude (degrees)
# \param lat2   Destination geographic latitude (degrees)
# \param lon2   Destination geographic longitude (degrees)
#
# \return Returns distance in meters.
#
def geomeasure(lat1, lon1, lat2, lon2):
  rlat1 = radians(lat1)
  rlon1 = radians(lon1)
  rlat2 = radians(lat1)
  rlon2 = radians(lon1)

  dLat = rlat2 - rlat1
  dLon = rlon2 - rlon1

  a = m.sin(dLat/2) * m.sin(dLat/2) + \
    m.cos(rlat1) * m.cos(rlat2) * \
    m.sin(dLon/2) * m.sin(dLon/2)

  c = 2 * m.atan2(m.sqrt(a), m.sqrt(1-a))
  d = Re * c

  return d * 1000.0 # meters

##
# \brief Calculate the geographic position at a small distance from the given
# position.
#
# The WGS84 spheroid model of the Earth is used. Only small delta distances
# are valid (<100km).
#
# \param lat    Starting geographic latitude (degrees)
# \param lon    Starting geographic longitude (degrees)
# \param dx     Delta position along N-S line (meters)
# \param dy     Delta position along W-E line (meters)
#
# \return New delta geographic latitude and longitude in degrees.
#
def geopos(lat, lon, dx, dy):
  rlat = radians(lat)
  rlon = radians(lon)

  m_per_deg_lat =   111132.92 \
                  - 559.82 * m.cos(2.0 * rlat) \
                  + 1.175 * m.cos(4.0 * rlat) \
                  - 0.0023 * m.cos(6.0 * rlat)

  m_per_deg_lon =   111412.84 * m.cos(rlat) \
                    - 93.5 * m.cos(3.0 * rlat) \
                    + 0.118 * m.cos(5.0 * rlat)

  dlat = dx/m_per_deg_lat
  dlon = dy/m_per_deg_lon

  return lat + dlat, lon + dlon

##
# \brief Non-interactively generate geographic feature coordinates from shapes.
#
# The shapes Json file is parsed along, with the optional feature collection
# Json file. If the feature collection file is not specified, a default Json
# template is used. The shapes are mapped to the specified geographic
# location and written as coordinates to a feature in the feature collection
# dictionary.
# 
# \param cliArgs  Command-line arguments.
#
# \return Feature collection dictionary with updated coordinates.
#
def goJson(cliArgs):
  shapes = readJson(cliArgs['shapesfile'])

  if cliArgs['fcfile']:
    fc = readJson(cliArgs['fcfile'])
  else:
    fc = OutDft.copy()

  k0 = 'starting_longitude'
  k1 = 'starting_latitude'
  if not shapes.has_key(k0):
    adios(EC_EXEC, "No {0} specified.".format(k0))
  elif BuiltInGeoCoords.has_key(shapes[k0]):
    shapes[k1], shapes[k0] = BuiltInGeoCoords[shapes[k0]]
  elif not shapes.has_key(k1):
    adios(EC_EXEC, "No {0} specified.".format(k1))

  k = 'scale'
  if shapes.has_key(k):
    shapes[k] = float(shapes[k])
  else:
    shapes[k] = InDft[k]

  k = 'rotate'
  if shapes.has_key(k):
    shapes[k] = float(shapes[k])
  else:
    shapes[k] = InDft[k]

  k = 'translate'
  if shapes.has_key(k):
    shapes[k] = [float(shapes[k][0]), float(shapes[k][1])]
  else:
    shapes[k] = InDft[k]

  k0 = 'path'
  k1 = 'paths'
  if shapes.has_key(k0):
    shapes[k1] = []
    shapes[k1].append(shapes[k0])
  elif shapes.has_key(k1):
    pass
  else:
    adios(EC_EXEC, "No {0} or {1} specified.".format(k0, k1))

  coord = processPaths(shapes, cliArgs)

  setFeatCoord(fc, cliArgs['feature'], coord)

  return fc

##
# \brief Interactively generate geographic feature coordinates from a shape.
#
# A geometric shape is gathered from user input. The shape is mapped to the
# specified geographic location and written as coordinates to a feature in
# the default feature collection dictionary.
# 
# \param cliArgs  Command-line arguments.
#
# \return Feature collection dictionary with updated coordinates.
#
def goInteractive(cliArgs):
  shapes = InDft.copy()
  fc     = OutDft.copy()

  s = raw_input('Enter starting longitude (degrees) or\n' \
                '  built-in position (name) [Boulder]: ')

  # Starting lon,lat with default Boulder
  if len(s) == 0:
    s = 'Boulder'
    lat0, lon0 = BuiltInGeoCoords[s]
    print "Using {0} as the starting location.".format(s)
  elif BuiltInGeoCoords.has_key(s):
    lat0, lon0 = BuiltInGeoCoords[s]
    print "Using {0} as the starting location.".format(s)
  else:
    lon0 = float(s)
    s = raw_input('Enter starting latitude (degrees): ')
    lat0 = float(s)

  print "Starting longitude,latitude {0},{1}".format(lon0, lat0)

  shapes['starting_longitude'] = lon0
  shapes['starting_latitude']  = lat0

  scale = 1.0
  s = raw_input('Enter scale [1.0]: ')
  if len(s) > 0:
    scale = float(s)
  shapes['scale'] = scale 

  translate = [0.0, 0.0]
  s = raw_input('Enter translate x y (meters) [0.0 0.0]: ')
  if len(s) > 0:
    args = s.split()
    translate = [float(args[0]), float(args[1])]
  shapes['translate'] = translate

  rotate = 0.0
  s = raw_input('Enter rotate (degrees) [0.0]: ')
  if len(s) > 0:
    rotate = float(s)
  shapes['rotate'] = rotate 

  print "2D affine transformation: " \
        "scale {0}, rotate {1}, translate {2}".format(scale, rotate, translate)

  print "Enter x,y meters from starting latitude,longitude.\n" \
        "The x+ is due North, y+ is due West\n" \
        "  ('q' to terminate path)"

  path = []
  while True:
    s = raw_input('x y (meters): ')
    args = s.split()
    if args[0] == 'q' or args[0] == 'Q':
      break
    x = float(args[0])
    y = float(args[1])
    path.append([x, y])
  shapes['paths'].append(path)

  coord = processPaths(shapes, cliArgs)

  setFeatCoord(fc, 0, coord)

  return fc

##
# \brief Print shapes dictionary to stdout.
#
# \param shapes   Shapes dictionary.
#
def printInput(shapes):
  print " ** Input:"
  print "Starting longitude: {0}".format(shapes['starting_longitude'])
  print "Starting latitude:  {0}".format(shapes['starting_latitude'])
  print "Scale:              {0}".format(shapes['scale'])
  print "Rotate:             {0}".format(shapes['rotate'])
  print "Translate:          {0}".format(shapes['translate'])
  print "Paths ({0}):".format(len(shapes['paths']))
  printPaths(shapes['paths'])

##
# \brief Print paths lists to stdout.
#
# \param paths  List of path lists.
#
def printPaths(paths):
  print "["
  m = len(paths)
  for i in range(m):
    print "  ["
    n = len(paths[i])
    for j in range(n):
      if j < n-1:
        print "    {0},".format(paths[i][j])
      else:
        print "    {0}".format(paths[i][j])
    if i < m-1:
      print "  ],"
    else:
      print "  ]"
  print "]"

##
# \brief Transform point x,y by applying a sequence of affine operations.
#
# \param  x       X coordinate.
# \param  y       Y coordinate.
# \param  affine  A list of affine matrices.
#
# \return Transformed point x', y'.
#
def transform(x, y, affine):
  augvec = np.matrix([[x], [y], [1.0]])
  for a in affine:
    augvec = a * augvec
  return augvec.item(0, 0), augvec.item(1, 0)

##
# \brief Map a list of paths to geographic location.
#
# \param shapes   Shapes dictionary.
# \param cliArgs  Command-line arguments.
#
# \return List of lists of geographic coordinates.
#
def processPaths(shapes, cliArgs):
  if cliArgs['trace']:
    printInput(shapes)

  lon0      = shapes['starting_longitude']
  lat0      = shapes['starting_latitude']
  scale     = shapes['scale']
  rotate    = radians(shapes['rotate'])
  translate = shapes['translate']

  # affine composing transformation matrices
  affine = []

  # rotate
  affine.append( np.matrix([
        [m.cos(rotate),  m.sin(rotate), 0],
        [-m.sin(rotate), m.cos(rotate), 0],
        [0.0, 0.0, 1.0]]) )

  # scale
  affine.append( np.matrix([
      [scale, 0.0, 0.0],
      [0.0, scale, 0.0],
      [0.0, 0.0, 1.0]]) )

  # translate
  affine.append( np.matrix([
      [1.0, 0.0, translate[0]],
      [0.0, 1.0, translate[1]],
      [0.0, 0.0, 1.0]]) )

  #affine = np.matrix([
  #    [scale * m.cos(rotate), scale * m.sin(rotate), translate[0]],
  #    [scale * -m.sin(rotate), scale * m.cos(rotate), translate[1]],
  #    [0.0, 0.0, 1.0]])

  transpaths = []
  geopaths   = []
  for path in shapes['paths']:
    tpath = []
    gpath = []
    for x, y in path:
      x_, y_ = transform(x, y, affine)
      tpath.append([x_, y_])
      lat, lon = geopos(lat0, lon0, x_, y_)
      gpath.append([lon, lat])
    gpath.append(gpath[0]) # close path
    transpaths.append(tpath)
    geopaths.append(gpath)

  if cliArgs['trace']:
    print "Transformed: "
    printPaths(transpaths)

  return geopaths


# ------------------------------------------------------------------------------
# Feature collection pretty-print routines
# ------------------------------------------------------------------------------

##
# \brief To comma or not to comma
#
def comma(last):
  if last:  return ''
  else:     return ','

##
# \brief Pretty-print a value.
#
# \param fp       Output file pointer.
# \param v        Value to print.
# \param indent   Left indentation.
# \param islast   This is [not] the last value in a container object.
#
def pprintVal(fp, v, indent, islast):
  if type(v) is str:
    print >>fp, '{0}"{1}"{2}'.format(' ' * indent, v, comma(islast))
  else:
    print >>fp, '{0}{1}{2}'.format(' ' * indent, v, comma(islast))

##
# \brief Pretty-print a dictionary value.
#
# \param fp       Output file pointer.
# \param k        Key to print.
# \param v        Value to print.
# \param indent   Left indentation.
# \param islast   This is [not] the last value in a container object.
#
def pprintDVal(fp, k, v, indent, islast):
  if type(v) is str:
    print >>fp, '{0}"{1}": "{2}"{3}'.format(' ' * indent, k, v, comma(islast))
  else:
    print >>fp, '{0}"{1}": {2}{3}'.format(' ' * indent, k, v, comma(islast))

##
# \brief Pretty-print a list.
#
# \param fp       Output file pointer.
# \param l        List.
# \param indent   Left indentation.
# \param islast   This is [not] the last value in a container object.
#
def pprintList(fp, l, indent, islast):
  sp0 = ' ' * indent
  sp1 = ' ' * (indent + 2)

  print >>fp, '{0}['.format(sp0)
  n = len(l) - 1
  i = 0
  for v in l:
    if type(v) is dict:
      pprintDict(fp, v, indent+2, i == n)
    elif type(v) is list:
      pprintList(fp, v, indent+2, i == n)
    else:
      pprintVal(fp, v, indent+2, i == n)
  print >>fp, '{0}]{1}'.format(sp0, comma(islast))

##
# \brief Pretty-print a dictionary.
#
# \param fp       Output file pointer.
# \param d        Dictionary to print.
# \param indent   Left indentation.
# \param islast   This is [not] the last value in a container object.
#
def pprintDict(fp, d, indent, islast):
  sp0 = ' ' * indent
  sp1 = ' ' * (indent + 2)
  print >>fp, '{0}{{'.format(sp0)
  n = len(d) - 1
  i = 0
  for k,v in d.items():
    if type(v) is dict:
      print >>fp, '{0}"{1}":'.format(sp1, k)
      pprintDict(fp, v, indent+2, i == n)
    elif type(v) is list:
      print >>fp, '{0}"{1}":'.format(sp1, k)
      pprintList(fp, v, indent+2, i == n)
    else:
      pprintDVal(fp, k, v, indent+2, i == n)
    i += 1
  print >>fp, '{0}}}{1}'.format(sp0, comma(islast))

##
# \brief Pretty-print the feature collection 'geometry' data.
#
# \param fp       Output file pointer.
# \param geo      Data to print.
# \param indent   Left indentation.
# \param islast   This is [not] the last value in a container object.
#
def pprintGeometry(fp, geo, indent):
  sp0   = ' ' * indent
  sp1   = ' ' * (indent + 2)
  sp2   = ' ' * (indent + 4)
  sp3   = ' ' * (indent + 6)

  print >>fp, '{0}{{'.format(sp0)

  print >>fp, '{0}"type": "{1}",'.format(sp1, geo['type'])
  print >>fp, '{0}"coordinates":'.format(sp1)

  print >>fp, '{0}['.format(sp1)
  coord = geo['coordinates']
  m = len(coord) - 1
  i = 0
  for polygon in coord:
    n = len(polygon) - 1
    j = 0
    print >>fp, '{0}['.format(sp2)
    for pt in polygon:
      print >>fp, '{0}[{1}, {2}]{3}'.format(sp3, pt[0], pt[1], comma(j == n))
      j += 1
    print >>fp, '{0}]{1}'.format(sp2, comma(i == m))
    i += 1
  print >>fp, '{0}]'.format(sp1)

  print >>fp, '{0}}}'.format(sp0)

##
# \brief Pretty-print a feature collection 'feature' data.
#
# \param fp       Output file pointer.
# \param feat     Data to print.
# \param indent   Left indentation.
# \param islast   This is [not] the last value in a container object.
#
def pprintFeature(fp, feat, indent, islast):
  sp0   = ' ' * indent
  sp1   = ' ' * (indent + 2)

  print >>fp, '{0}{{'.format(sp0)
  print >>fp, '{0}"type": "{1}",'.format(sp1, feat['type'])

  print >>fp, '{0}"properties":'.format(sp1)
  pprintDict(fp, feat['properties'], indent+2, False)

  print >>fp, '{0}"geometry":'.format(sp1)
  pprintGeometry(fp, feat['geometry'], indent+2)

  print >>fp, '{0}}}{1}'.format(sp0, comma(islast))

##
# \brief Pretty-print a feature collection Json file.
#
# \param fc       Feature collection dictionary.
# \param cliArgs  Command-line arguments.
#
def pprintFeatureCollection(fc, cliArgs):
  if cliArgs['output']:
    try:
      fp = open(cliArgs['output'], 'w')
    except IOError:
      adios(EC_EXEC, "Cannot open file {0}".format(cliArgs['output']))
  else:
    fp = sys.stdout

  print >>fp, '{'
  print >>fp, '  "type": "{0}",'.format(fc['type'])
  print >>fp, '  "features":'
  print >>fp, '  ['
  n = len(fc['features']) - 1
  i = 0
  for feat in fc['features']:
    pprintFeature(fp, feat, 4, i == n)
    i += 1
  print >>fp, "  ]"
  print >>fp, "}"

  if cliArgs['output']:
    fp.close()


# ------------------------------------------------------------------------------
# The main
# ------------------------------------------------------------------------------
if __name__ == '__main__':
  argv = sys.argv

  # command-line interface arguments dictionary
  cliArgs = {}

  # defaults
  cliArgs['shapesfile'] = None
  cliArgs['fcfile']     = None
  cliArgs['feature']    = 0
  cliArgs['output']     = None
  cliArgs['trace']      = False

  # parse command-line options and arguments
  try:
    opts, args = getopt.gnu_getopt(argv[1:], "f:o:t?h",
        ['feature=', 'output=', 'trace', 'help'])
  except getopt.error, msg:
    adios(EC_ARGS, "Unknown option. See '{0} --help'".format(Argv0))

  # set options
  for opt, optarg in opts:
    if opt in ('-f', '--feature'):
      try:
        cliArgs['feature'] = int(optarg)
      except ValueError:
        adios(EC_ARGS,
            "'{0}' argument '{1}' not an integer.".format(opt, optarg))
    elif opt in ('-o', '--output'):
      cliArgs['output'] = optarg
    elif opt in ('-t', '--trace'):
      cliArgs['trace'] = True
    elif opt in ('-h', '--help', '-?'):
      printHelp()
      adios(EC_OK)

  # set arguments
  if len(args) > 0:
    cliArgs['shapesfile'] = args[0]
    if len(args) > 1:
      cliArgs['fcfile'] = args[1]

  # read input shapes and return generated feature collection
  if cliArgs['shapesfile']:
    fc = goJson(cliArgs)
  else:
    fc = goInteractive(cliArgs)

  # print feature collection
  if cliArgs['trace']:
    print
    print "#### Feature Collection Output ####"

  pprintFeatureCollection(fc, cliArgs)
