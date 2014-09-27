import Queue
from collections import defaultdict
import math

from shapely.geometry.linestring import LineString

from nonholonomic_shortest_path import geom_util
from nonholonomic_shortest_path.geom_util import Circle, CLOCKWISE, \
  COUNTER_CLOCKWISE, ANGLE_MOD, EPS


MAX_TURNING_ANGLE = math.pi / 4.0
VEHICLE_LENGTH = 0.1
TURNING_RADIUS = VEHICLE_LENGTH / math.sin(MAX_TURNING_ANGLE)


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
               direction,  # geom_util.CLOCKWISE or geom_util.COUNTER_CLOCKWISE
               origin_configuration=None,
               is_init=False,
               is_goal=False,
               point=None,  # point of the thing. Applicable only on is_init and is_goal.
               ):
    self.pk = pk
    self.best_value = None
    self.processed_value = None
    self.previous_config = None
    assert direction in [CLOCKWISE, COUNTER_CLOCKWISE]
    assert origin_configuration or is_init or is_goal
    if point:
      assert is_init or is_goal
    else:
      assert origin_configuration
    if origin_configuration:
      assert point is None

    self.circle = circle
    self.direction = direction
    self.origin_configuration = origin_configuration
    self.is_init = is_init
    self.is_goal = is_goal

    if is_init or is_goal:
      self.angle = circle.PointToAngle(point)
    else:
      self.origin_angle, self.angle = geom_util.GetTangentLine(
          origin_configuration.circle,
          origin_configuration.direction,
          circle,
          direction)


class ConfigurationManager(object):
  def __init__(self):
    self.configurations = []  # List of all configurations
    self.circle_end_configs = defaultdict(list)  # list of final configurations on this circle.
    self.circle_pair_to_config = {}  # maps (origin_circle_pk, ccw, dest_circle_pk, ccw) to config if any.

  def GetInitialConfigurations(self):
    return [config for config in self.configurations if config.is_init]

  def GetTwoCirclesConfig(self,
                          origin_config,
                          target_circle,
                          target_direction):
    key = (origin_config.circle.pk, target_circle, origin_config.direction, target_direction,)
    if key in self.circle_pair_to_config:
      return self.circle_pair_to_config[key]
    config = Configuration(len(self.configurations),
                           target_circle,
                           target_direction,
                           origin_config)
    self.configurations.append(config)
    self.circle_pair_to_config[key] = config
    return config

  def CreateInitConfig(self, circle, direction, start_point):
    config = Configuration(
        len(self.configurations),
        circle,
        direction,
        is_init=True,
        point=start_point)
    self.configurations.append(config)

  def CreateEndConfig(self, circle, direction, goal_point):
    config = Configuration(
        len(self.configurations),
        circle,
        direction,
        is_goal=True,
        point=goal_point)
    self.configurations.append(config)
    self.circle_end_configs[circle.pk].append(config)


def _AdjCircles(configuration, radius):
  '''Generate the two indexed adjacent circles to the configuration.'''
  angles = [(configuration[1] + math.pi / 2.0) % ANGLE_MOD,
            (configuration[1] - math.pi / 2.0) % ANGLE_MOD]
  circles = []
  for angle in angles:
    circles.append(IndexedCircle(center=(configuration[0][0] + math.cos(angle) * radius,
                                         configuration[0][1] + math.sin(angle) * radius),
                                 radius=radius))
  return circles


def _GenerateCircle(linestring, angle_fraction, radius):
  '''Generate a circle on the angle represented by linestring with three points.
  The center of the circle should be angle_fraction away from the boundary.
  It will touch the vertex.'''
  circle = geom_util.Circle(center=linestring.coords[1],
                            radius=radius)
  begin_angle = circle.PointToAngle(linestring.coords[2])
  end_angle = circle.PointToAngle(linestring.coords[0])
  delta = (end_angle - begin_angle) % ANGLE_MOD
  at_angle = (begin_angle + angle_fraction * delta) % ANGLE_MOD
  return IndexedCircle(center=circle.AngleToPoint(at_angle),
                       radius=radius)


def _GenerateCirclesAndBasicConfig(start_config,
                                   goal_config,
                                   obstacles,
                                   turning_radius,
                                   manager,):
  circles = []
  # First generate two circles adjacent to the start and goal configs.
  circles.extend(_AdjCircles(start_config, turning_radius))
  circles.extend(_AdjCircles(goal_config, turning_radius))
  manager.CreateInitConfig(circles[0], COUNTER_CLOCKWISE, start_config[0])
  manager.CreateInitConfig(circles[1], CLOCKWISE, start_config[0])
  manager.CreateEndConfig(circles[2], COUNTER_CLOCKWISE, goal_config[0])
  manager.CreateEndConfig(circles[3], CLOCKWISE, goal_config[0])

  # Now generate midpoint circle on each vertex.
  for obstacle in obstacles:
    coords = obstacle.exterior.coords
    # Not a bug, the center is really only n-1 elements. This is since polygon's boundary repeats the last vertex.
    for p1, p2, p3 in zip(coords, coords[1:], coords[2:-1] + coords[:2]):
      for fractions in [0.0, 0.5, 1.0]:
        circles.append(_GenerateCircle(
            LineString([p1, p2, p3]),
            angle_fraction=fractions,
            radius=turning_radius))

  # TODO(irvan): generate random circles
  return circles


def Heuristic(config, goal_config):
  return Distance(config.circle.center,
                  goal_config[0])


class ObstacleManager(object):
  def __init__(self, obstacles):
    self.obstacles = obstacles

  def CanArcGo(self, arc):
    for obstacle in self.obstacles:
      if geom_util.IsArcStrictIntersectPoly(arc, obstacle):
        return False
    return True

  def CanLineStringGo(self, linestring):
    for obstacle in self.obstacles:
      if linestring.crosses(obstacle) or obstacle.contains(linestring):
        return False
    return True


def ConstructPath(start_config,
                  goal_config,
                  obstacles,
                  turning_radius=TURNING_RADIUS,
                  ):
  '''
  Construct a path consisting of segments and arcs from start_config to goal_config
  if any. Otherwise returns None.

  Returns a sequence of geom_util.Arc and sympy.Segment.

  Complexity is around N^5, but using A* should fasten things up.

  It is assumed that if the boundaries of the field is given implicitly as
  obstacles.'''
  manager = ConfigurationManager()
  obstacle_manager = ObstacleManager(obstacles)
  print 'Generating circles.'
  circles = _GenerateCirclesAndBasicConfig(start_config,
                                           goal_config,
                                           obstacles,
                                           turning_radius,
                                           manager)
  print '{0} circles generated'.format(len(circles))

  # Next, run A* starting from the initial circle.
  queue = Queue.PriorityQueue()

  def PutToQueue(config, new_value, previous_config, paths=None):
    if config.best_value is not None and config.best_value < new_value + EPS:
      return
    config.best_value = new_value
    config.previous_config = previous_config
    config.paths = paths
    queue.put((new_value + Heuristic(config, goal_config),
               config))

  for init_config in manager.GetInitialConfigurations():
    PutToQueue(init_config, new_value=0.0, previous_config=None)

  found_goal = None
  print 'Running A*'
  while not queue.empty():
    _, config = queue.get()
    assert config.best_value is not None

    if config.processed_value is not None and abs(config.processed_value - config.best_value) <= EPS:
      # Already processed.
      continue
    # print 'Processing node {0} {1}'.format(config.circle.center, config.previous_config)
    config.processed_value = config.best_value

    if config.is_goal:
      # Found it
      found_goal = config
      break

    for goal in manager.circle_end_configs[config.circle.pk]:
      assert goal.circle.pk == config.circle.pk
      if goal.direction != config.direction:
        continue
      arc = geom_util.Arc(center=goal.circle.center,
                          radius=goal.circle.radius,
                          begin_radian=config.angle,
                          end_radian=goal.angle,
                          direction=goal.direction)
      if obstacle_manager.CanArcGo(arc):
        PutToQueue(goal, config.best_value + arc.Length(), config, paths=[arc])

    for circle in circles:
      if circle.pk == config.circle.pk:
        continue  # Prevent moving in the same circle.

      for direction in [COUNTER_CLOCKWISE, CLOCKWISE]:
        if direction != config.direction and geom_util.CircleIntersects(circle, config.circle):
          continue  # not possible to change direction on intersecting circles.

        next_config = manager.GetTwoCirclesConfig(
            config, circle, direction)
        arc = geom_util.Arc(center=config.circle.center,
                            radius=config.circle.radius,
                            begin_radian=config.angle,
                            end_radian=next_config.origin_angle,
                            direction=config.direction)
        segment = LineString([config.circle.AngleToPoint(next_config.origin_angle),
                              next_config.circle.AngleToPoint(next_config.angle)])
        if obstacle_manager.CanArcGo(arc) and obstacle_manager.CanLineStringGo(segment):
          PutToQueue(next_config, config.best_value + arc.Length() + segment.length, config, paths=[arc, segment])

  if not found_goal:
    return None
  else:
    print 'Backtracking'
    paths = []
    print 'Found path of length {0}'.format(found_goal.best_value)
    while found_goal.previous_config is not None:
      paths = found_goal.paths + paths
      found_goal = found_goal.previous_config
    return paths

