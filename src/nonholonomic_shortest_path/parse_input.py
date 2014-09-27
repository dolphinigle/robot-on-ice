'''config is of the form (sympy.Point, orientation).
   orientation is radian wrt positive x axis.'''

from decimal import Decimal
import exceptions

from sympy.geometry.point import Point
from sympy.geometry.polygon import Polygon


def ReadInput(filename):
  '''Read input file named filename

  Returns a tuple (start_config, end_config, obstacles)

  Obstacles are sympy.Polygon objects.

  >>> ReadInput('../../inputs/test_basic.mp')
  ((Point(1/10, 1/10), 0.0), (Point(9/10, 9/10), 1.57), [Polygon(Point(1/10, 1/5), Point(1/10, 3/10), Point(9/10, 3/10), Point(9/10, 1/5)), Polygon(Point(2/5, 1/2), Point(2/5, 3/5), Point(1/2, 3/5), Point(1/2, 1/2))])
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
      config = (Point(x, y), orientation)
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
        points.append(Point(*map(Decimal, lines[i].strip().split())))
        i += 1
      i += 1
      obstacles.append(Polygon(*points))
    else:
      raise exceptions.IOError('Keyword {0} not recognized'.format(lines[i]))

  return (start_config, goal_config, obstacles)


