import Queue
from collections import defaultdict
import math
import random

from shapely import ops
from shapely.geometry.linestring import LineString
from shapely.geometry.multipolygon import MultiPolygon
from shapely.geometry.point import Point
from shapely.geometry.polygon import Polygon

from nonholonomic_shortest_path import geom_util
from nonholonomic_shortest_path.geom_util import Circle, ANGLE_MOD, EPS, \
  CLOCKWISE, COUNTER_CLOCKWISE


HEURISTIC_EUCLID = 'EUCLID'
HEURISTIC_VIS = 'VIS'
HEURISTIC = HEURISTIC_EUCLID
MAX_TURNING_ANGLE = math.pi / 4.0
VEHICLE_LENGTH = 0.1
TURNING_RADIUS = VEHICLE_LENGTH / math.sin(MAX_TURNING_ANGLE)
BOUNDARIES = [-0.5, 1.5]


class IndexedCircle(Circle):
  pk_counter = 0
  def __init__(self, *args, **kwargs):
    super(IndexedCircle, self).__init__(*args, **kwargs)
    self.pk = IndexedCircle.pk_counter
    IndexedCircle.pk_counter += 1


def Distance(point1, point2):
  return math.sqrt((point1[0] - point2[0])**2 + (point1[1] - point2[1])**2)


class Configuration(object):
  '''Represents a particular configuration in the configuration space.'''
  def __init__(self,
               pk,
               circle,
               angle,
               direction,
               is_init=False,
               is_goal=False,
               ):
    self.pk = pk
    self.best_value = None
    self.processed_value = None
    self.previous_config = None

    self.circle = circle
    self.is_init = is_init
    self.is_goal = is_goal

    self.angle = angle
    self.direction = direction


class ConfigurationManager(object):
  def __init__(self):
    self.configurations = []  # List of all configurations
    self.circle_end_configs = defaultdict(list)  # list of final configurations on this circle.
    self.circle_pair_to_config = {}  # maps (origin_circle_pk, ccw, dest_circle_pk, ccw) to config if any.

  def GetInitialConfigurations(self):
    return [config for config in self.configurations if config.is_init]

  def GetTwoCirclesConfig(self,
                          circle,
                          target_circle,
                          origin_dir,
                          target_dir,
                          obstacle_manager,
                          direction):
    key = (circle.pk, target_circle.pk, origin_dir, target_dir, direction)
    if key in self.circle_pair_to_config:
      return self.circle_pair_to_config[key]
    angle1, angle2 = geom_util.GetTangentLine(circle, origin_dir,
                                              target_circle, target_dir)

    if not obstacle_manager.CanLineStringGo(LineString([
        circle.AngleToPoint(angle1),
        target_circle.AngleToPoint(angle2)])):
      self.circle_pair_to_config[key] = None
      return None

    config = Configuration(len(self.configurations),
                           target_circle,
                           angle2,
                           direction,)
    self.configurations.append(config)
    self.circle_pair_to_config[key] = (config, angle1)
    return config, angle1

  def CreateInitConfig(self, circle, start_point, direction):
    config = Configuration(
        len(self.configurations),
        circle,
        angle=circle.PointToAngle(start_point),
        direction=direction,
        is_init=True)
    self.configurations.append(config)

  def CreateEndConfig(self, circle, goal_point, direction):
    config = Configuration(
        len(self.configurations),
        circle,
        angle=circle.PointToAngle(goal_point),
        direction=direction,
        is_goal=True,)
    self.configurations.append(config)
    self.circle_end_configs[circle.pk].append(config)


def _AdjCircles(configuration, radius, level):
  '''Generate the two indexed adjacent circles to the configuration.'''
  angles = [(configuration[1] + math.pi / 2.0) % ANGLE_MOD,
            (configuration[1] - math.pi / 2.0) % ANGLE_MOD,
            ]

  if level >= 1:
    delta = 1.0 / (2**level)
    for i in range(2**level - 1):
      angles.append((configuration[1] + math.pi / 2.0 + (i+1) * delta * math.pi) % ANGLE_MOD)
      angles.append((configuration[1] - math.pi / 2.0 + (i+1) * delta * math.pi) % ANGLE_MOD)

  circles = []
  for angle in angles:
    circles.append(IndexedCircle(center=(configuration[0][0] + math.cos(angle) * radius,
                                         configuration[0][1] + math.sin(angle) * radius),
                                 radius=radius))

  return circles


def _GenerateCircle(linestring, angle_fraction, radius, mirror=False):
  '''Generate a circle on the angle represented by linestring with three points.
  The center of the circle should be angle_fraction away from the boundary.
  It will touch the vertex.'''
  circle = geom_util.Circle(center=linestring.coords[1],
                            radius=radius)
  begin_angle = circle.PointToAngle(linestring.coords[0])
  end_angle = circle.PointToAngle(linestring.coords[2])

  real_begin_angle = (begin_angle - math.pi / 2.0) % ANGLE_MOD
  real_end_angle = (end_angle + math.pi / 2.0) % ANGLE_MOD
  begin_angle = real_begin_angle
  end_angle = real_end_angle

  delta = (end_angle - begin_angle) % ANGLE_MOD

  at_angle = (begin_angle + angle_fraction * delta) % ANGLE_MOD
  if mirror:
    at_angle = (at_angle + math.pi) % ANGLE_MOD

  return IndexedCircle(center=circle.AngleToPoint(at_angle),
                       radius=radius)


def _GenerateCirclesAndBasicConfig(start_config,
                                   goal_config,
                                   obstacles,
                                   turning_radius,
                                   manager,
                                   level):
  circles = []
  # First generate two circles adjacent to the start and goal configs.
  begins = _AdjCircles(start_config, turning_radius, level)
  ends = _AdjCircles(goal_config, turning_radius, level)
  for i in range(2):
    manager.CreateInitConfig(begins[i], start_config[0], 1 - i)
    manager.CreateEndConfig(ends[i], goal_config[0], 1 - i)
  circles.extend(begins)
  circles.extend(ends)

  if level == 0:
    fracts = [0.5]
  else:
    delta = 1.0 / (2**level)
    fracts = [delta]
    while fracts[-1] + delta < 1.0 - EPS:
      fracts.append(fracts[-1] + delta)
    fracts = [0.0] + fracts + [1.0]

  # Now generate midpoint circle on each vertex.
  for polygon in obstacles.geoms:
    for coords in list(map(lambda interior: interior.coords, polygon.interiors)) + [polygon.exterior.coords]:
      # Not a bug, the center is really only n-1 elements. This is since polygon's boundary repeats the last vertex.
      for p3, p2, p1 in zip(coords, coords[1:], coords[2:-1] + coords[:2]):
        if p2[0] < BOUNDARIES[0] or p2[0] > BOUNDARIES[1] or p2[1] < BOUNDARIES[0] or p2[1] > BOUNDARIES[1]:
          continue
        if geom_util.CrossProd(p3, p2, p1) > -EPS:
          continue
        for fractions in fracts:
          circles.append(_GenerateCircle(
              LineString([p1, p2, p3]),
              angle_fraction=fractions,
              radius=turning_radius))
          circles.append(_GenerateCircle(
              LineString([p1, p2, p3]),
              angle_fraction=fractions,
              radius=turning_radius,
              mirror=True))
  
  # TODO(irvan): generate random circles
  return circles


def EuclidHeuristic(config, goal_config, *args, **kwargs):
  return Distance(config.circle.AngleToPoint(config.angle),
                  goal_config[0])


def VisibilityGraphHeuristic(config, goal_config, vis_graph):
  return vis_graph.HeuristicSlow(config.circle.AngleToPoint(config.angle))
  # return vis_graph.Heuristic(config.circle) - config.circle.radius


def Heuristic(config, goal_config, *args, **kwargs):
  if HEURISTIC == HEURISTIC_EUCLID:
    return EuclidHeuristic(config, goal_config, *args, **kwargs)
  else:
    return VisibilityGraphHeuristic(config, goal_config, *args, **kwargs)


class VisibilityGraph(object):
  def __init__(self, obstacles, goal_point, obstacle_manager):
    # Gather all points
    point_set = []
    for poly in obstacles:
      for coords in map(lambda i: i.coords, poly.interiors) + [poly.exterior.coords]:
        for p1, p2, p3 in zip(coords, coords[1:], coords[2:-1] + coords[:2]):
          if geom_util.CrossProd(p1, p2, p3) < -EPS:
            point_set.append(p2)
    random.shuffle(point_set)

    point_set = [goal_point] + point_set

    n = len(point_set)
    distances = [None] * n

    queue = Queue.PriorityQueue()
    distances[0] = 0.0
    queue.put((0.0, 0))
    while not queue.empty():
      dist, idx = queue.get()
      if distances[idx] + EPS < dist:
        continue  # deprecated branch
      for i in range(n):
        if distances[i] is None or distances[i] > distances[idx]:
          euclid_dist = math.sqrt((point_set[i][0] - point_set[idx][0])**2 +
                                  (point_set[i][1] - point_set[idx][1])**2)
          if distances[i] is None or distances[i] > EPS + distances[idx] + euclid_dist:
            if obstacle_manager.CanLineStringGo(LineString([point_set[i], point_set[idx]])):
              distances[i] = distances[idx] + euclid_dist
              queue.put((distances[i], i))
    self.distances = distances
    self.points = point_set
    self.obstacle_manager = obstacle_manager
    self.circle_cache = {}
    self.goal_point = goal_point


  def HeuristicSlow(self, point):
    best = None
    for p, dist in zip(self.points, self.distances):
      if dist is not None:
        if best is None or dist < best:
          euc = math.sqrt((p[0] - point[0])**2 + (p[1] - point[1])**2)
          if best is None or best > dist + euc:
            if self.obstacle_manager.CanLineStringGo(LineString([p, point])):
              best = dist + euc
    if best is None:
      best = 10**9
    return best



  def Heuristic(self, circle):
    if circle.pk in self.circle_cache:
      return self.circle_cache[circle.pk]

    if self.obstacle_manager.obstacles.contains(Point(circle.center[0],
                                                      circle.center[1])):
      # :O
      best = Distance(circle.center, self.goal_point)
      self.circle_cache[circle.pk] = best
      return best

    point = circle.center
    best = None
    for p, dist in zip(self.points, self.distances):
      if dist is not None:
        if best is None or dist < best:
          euc = math.sqrt((p[0] - point[0])**2 + (p[1] - point[1])**2)
          if best is None or best > dist + euc:
            if self.obstacle_manager.CanLineStringGo(LineString([p, point])):
              best = dist + euc
    if best is None:
      best = 10**9
    self.circle_cache[circle.pk] = best
    return best


class ObstacleManager(object):
  def __init__(self, obstacles, circles):
    self.obstacles = obstacles
    self.circles = circles

    # Calculate the intersecting thingies for each circle.
    for circle in circles:
      intersections = []
      for obstacle in obstacles.geoms:
        intersections.extend(geom_util.CirclePolygonIntersections(circle, obstacle))

      angles = []
      for intersection in intersections:
        angles.append(circle.PointToAngle(intersection))

      if len(angles) == 0:
        angles.append(0.0)

      angles.sort()

      # Remove angles that are approximately equal
      cleaned_angle = [angles[0]]
      for angle1, angle2 in zip(angles, angles[1:]):
        if abs(angle2 - angle1) > EPS:
          cleaned_angle.append(angle2)
      angles = cleaned_angle

      is_obstructed = []
      for p1, p2 in zip(angles, angles[1:] + angles[:1]):
        dist = (p2 - p1) % (math.pi * 2.0)
        midangle = (p1 + dist / 2.0) % (math.pi * 2.0)
        point = circle.AngleToPoint(midangle)
        point = Point(point[0], point[1])
        obstructed = self.obstacles.contains(point)
        is_obstructed.append(obstructed)

      circle.intersection_angles = angles
      circle.intersection_is_obstructed = is_obstructed

    self.circle_pk_to_circle = {}
    for circle in circles:
      self.circle_pk_to_circle[circle.pk] = circle

    self.tangent_cache = {}

  def CanArcGo(self, arc):
    assert arc.circle_pk is not None
    circle = self.circle_pk_to_circle[arc.circle_pk]
    for angle1, angle2, is_obstructed in zip(
        circle.intersection_angles,
        circle.intersection_angles[1:] + circle.intersection_angles[:1],
        circle.intersection_is_obstructed):
      if is_obstructed:
        if (geom_util.IsAngleBetween(arc._begin_radian, angle1, angle2) or
            geom_util.IsAngleBetween(arc._end_radian, angle1, angle2) or
            geom_util.IsAngleBetween(angle1, arc._begin_radian, arc._end_radian) or
            geom_util.IsAngleBetween(angle2, arc._begin_radian, arc._end_radian) or
            geom_util.AngleAlmostEqual(arc._begin_radian, angle1) or
            geom_util.AngleAlmostEqual(arc._end_radian, angle2)
            ):
          return False
    return True

  def CanLineStringGo(self, linestring):
    return not linestring.crosses(self.obstacles) and not self.obstacles.contains(linestring)

def ConstructPath(start_config,
                  goal_config,
                  obstacles,
                  boundary_obstacles,
                  level,  # Level 0 is fastest, 1 is twice slower, 2 is 4 times slower, and so forth.
                  turning_radius=TURNING_RADIUS,
                  ):
  '''
  Construct a path consisting of segments and arcs from start_config to goal_config
  if any. Otherwise returns None.

  Returns a sequence of geom_util.Arc and sympy.Segment.

  Complexity is around N^5, but using A* should fasten things up.

  It is assumed that if the boundaries of the field is given implicitly as
  obstacles.'''
  obstacles = ops.cascaded_union(obstacles + boundary_obstacles)
  if isinstance(obstacles, Polygon):
    obstacles = MultiPolygon([obstacles])

  manager = ConfigurationManager()
  circles = _GenerateCirclesAndBasicConfig(start_config,
                                           goal_config,
                                           obstacles,
                                           turning_radius,
                                           manager,
                                           level)

  print '{0} circles generated. Running A*...'.format(len(circles))
  obstacle_manager = ObstacleManager(obstacles, circles)

  if HEURISTIC == HEURISTIC_EUCLID:
    vis_graph = None
  else:
    vis_graph = VisibilityGraph(obstacles, goal_config[0], obstacle_manager)

  # Next, run A* starting from the initial circle.
  queue = Queue.PriorityQueue()

  def PutToQueue(config, new_value, previous_config, paths=None):
    if config.best_value is not None and config.best_value < new_value + EPS:
      return
    config.best_value = new_value
    config.previous_config = previous_config
    config.paths = paths
    queue.put((new_value + Heuristic(config, goal_config, vis_graph),
               config))

  for init_config in manager.GetInitialConfigurations():
    PutToQueue(init_config, new_value=0.0, previous_config=None)

  found_goal = None
  expansions = 0
  re_expansions = 0
  while not queue.empty():
    _, config = queue.get()
    assert config.best_value is not None
    expansions += 1

    if config.processed_value is not None and config.best_value + EPS >= config.processed_value:
      # Already processed.
      continue

    if config.processed_value is not None:
      re_expansions += 1

    config.processed_value = config.best_value

    if config.is_goal:
      # Found it
      found_goal = config
      break

    for goal in manager.circle_end_configs[config.circle.pk]:
      assert goal.circle.pk == config.circle.pk
      if goal.direction != config.direction:
        continue
      for direction in [COUNTER_CLOCKWISE, CLOCKWISE]:
        arc = geom_util.Arc(center=goal.circle.center,
                            radius=goal.circle.radius,
                            begin_radian=config.angle,
                            end_radian=goal.angle,
                            direction=direction,
                            circle_pk=goal.circle.pk)
        if obstacle_manager.CanArcGo(arc):
          PutToQueue(goal, config.best_value + arc.Length(), config, paths=[arc])

    for circle in circles:
      if circle.pk == config.circle.pk:
        continue  # Prevent moving in the same circle.

      # Try all four combinations
      for init_direction in [COUNTER_CLOCKWISE, CLOCKWISE]:
        for target_direction in [COUNTER_CLOCKWISE, CLOCKWISE]:
          if init_direction != target_direction and geom_util.CircleIntersects(config.circle, circle):
            continue
          if init_direction == config.direction:
            new_dir = target_direction
          else:
            new_dir = 1 - target_direction
          val = manager.GetTwoCirclesConfig(
              config.circle, circle, init_direction, target_direction, obstacle_manager, new_dir)
          if not val:
            continue
          next_config, angle = val

          # First attempt the arc movement.
          arcs = []
          for arc_direction in [COUNTER_CLOCKWISE, CLOCKWISE]:
            arcs.append(geom_util.Arc(center=config.circle.center,
                                      radius=config.circle.radius,
                                      begin_radian=config.angle,
                                      end_radian=angle,
                                      direction=arc_direction,
                                      circle_pk=config.circle.pk))
          valids = [arc for arc in arcs if obstacle_manager.CanArcGo(arc)]

          if not valids:
            continue

          best_arc = valids[0]
          if len(valids) == 2 and valids[1].Length() < valids[0].Length():
            best_arc = valids[1]

          segment = LineString([config.circle.AngleToPoint(angle),
                                next_config.circle.AngleToPoint(next_config.angle)])

          new_value = config.best_value + best_arc.Length() + segment.length
          if next_config.best_value is None or next_config.best_value > new_value + EPS:
            PutToQueue(next_config, new_value, config, paths=[best_arc, segment])
    
  print 'Expanded {0} nodes. {1} re-expansions.'.format(expansions, re_expansions)
  if not found_goal:
    return None, None
  else:
    paths = []
    length = found_goal.best_value
    while found_goal.previous_config is not None:
      paths = found_goal.paths + paths
      found_goal = found_goal.previous_config
    return paths, length

