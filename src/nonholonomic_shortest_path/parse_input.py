'''config is of the form (sympy.Point, orientation).
   orientation is radian wrt positive x axis.'''

from shapely.geometry.linestring import LineString
from shapely.geometry.polygon import Polygon

from nonholonomic_shortest_path import geom_util
from nonholonomic_shortest_path.geom_util import EPS


def ReadInput(filename, enlargement=0.0):
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
  settings = {}
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
      if len(points) == 4:
        # Enlarge them by a wee bit.
        xs = [p[0] for p in points]
        ys = [p[1] for p in points]
        for j in range(4):
          if abs(points[j][0] - max(xs)) < EPS:
            points[j][0] += enlargement
          if abs(points[j][0] - min(xs)) < EPS:
            points[j][0] -= enlargement
          if abs(points[j][1] - max(ys)) < EPS:
            points[j][1] += enlargement
          if abs(points[j][1] - min(ys)) < EPS:
            points[j][1] -= enlargement
      obstacles.append(Polygon(points))
    else:
      assert lines[i][0] == '<' and lines[i][-1] == '>'
      settings[lines[i][1:-1]] = float(lines[i+1])
      i += 3

  return (start_config, goal_config, obstacles, settings)


