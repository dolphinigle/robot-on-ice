'''config is of the form (sympy.Point, orientation).
   orientation is radian wrt positive x axis.'''

import exceptions

from shapely.geometry.linestring import LineString
from shapely.geometry.polygon import Polygon

from nonholonomic_shortest_path import geom_util


def ReadInput(filename):
  '''Read input file named filename

  Returns a tuple (start_config, end_config, obstacles)

  Obstacles are shapely.Polygon objects.

  '''
  f = open(filename, 'r')
  lines = f.readlines()

  i = 0

  start_config = None
  goal_config = None
  obstacles = []
  lines = map(lambda x: x.strip(), lines)
  while i < len(lines):
    if not lines[i]:
      i += 1
      continue
    if lines[i] == '<Start>' or lines[i] == '<Goal>':
      x, y, orientation = map(float, lines[i+1].split())
      config = ((x, y), orientation)
      if lines[i] == '<Start>':
        assert start_config is None
        assert lines[i+2] == '</Start>'
        start_config = config
      else:
        assert goal_config is None
        assert lines[i+2] == '</Goal>'
        goal_config = config
      i += 3
    elif lines[i] == '<Obstacle>':
      i += 1
      points = []
      while lines[i] != '</Obstacle>':
        points.append((map(float, lines[i].strip().split())))
        i += 1
      i += 1
      if not geom_util.IsCcw(LineString(points)):
        points.reverse()
      obstacles.append(Polygon(points))
    else:
      raise exceptions.IOError('Keyword {0} not recognized'.format(lines[i]))

  return (start_config, goal_config, obstacles)


