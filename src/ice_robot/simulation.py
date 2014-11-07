import math
import time

from shapely import ops
from shapely.geometry.polygon import Polygon

from ice_robot import ice_engine
from ice_robot.markov import AXIS_RESOLUTION, ORIENTATION_RESOLUTION, \
  MAX_TURNING_ANGLE, VEHICLE_LENGTH, AXIS_VARIANCE, ORIENTATION_VARIANCE, \
  ICED_REWARD, GOAL_REWARD
from nonholonomic_shortest_path import parse_input, draw
from nonholonomic_shortest_path.geom_util import ANGLE_MOD


PATHS_TO_SIMULATE = 5
BG_PATHS_TO_SIMULATE = 100
PATHS_TO_SIMULATE_FOR_STATS = 1000
WALK_LIMIT = 500


def Simulate(filename, engine_class):
  start_config, goal_config, obstacles, settings = parse_input.ReadInput(filename)
  eps = 0.01
  additional_obstacles = [
      Polygon([(0 - eps, 0 - eps),
               (0 - eps, 2),
               (-1, 2),
               (-1, -1),
               (2, -1),
               (2, 2),
               (1 + eps, 2),
               (1 + eps, 0 - eps)]),
      Polygon([(2, 1 + eps),
               (2, 2),
               (-1, 2),
               (-1, 1 + eps)])]
  start_time = time.clock()
  all_obstacles = ops.cascaded_union(obstacles + additional_obstacles)
  print all_obstacles

  axis_resolution = int(settings.get('AxisResolution', AXIS_RESOLUTION))
  orientation_resolution = int(settings.get('OrientationResolution', ORIENTATION_RESOLUTION))
  step_length = 1.0 / axis_resolution * (2**0.5)
  discount = settings.get('Discount', 0.95)

  engine = engine_class(
      goal_config=goal_config,
      obstacles=all_obstacles,
      max_steering_angle=MAX_TURNING_ANGLE,
      vehicle_length=VEHICLE_LENGTH,
      x_variance=settings.get('AxisVariance', AXIS_VARIANCE),
      y_variance=settings.get('AxisVariance', AXIS_VARIANCE),
      orientation_variance=settings.get('OrientationVariance', ORIENTATION_VARIANCE),
      average_path_length=step_length,
      axis_resolution=axis_resolution,
      orientation_resolution=orientation_resolution,
      iced_reward=settings.get('IcedReward', ICED_REWARD),
      goal_reward=settings.get('GoalReward', GOAL_REWARD),
      discount=discount,
  )

  ideal_path, _ = ice_engine.CreatePath(start_config, engine, all_obstacles, WALK_LIMIT, ideal=True)
  paths = []
  for _ in range(PATHS_TO_SIMULATE):
    p, _ = ice_engine.CreatePath(start_config, engine, all_obstacles, WALK_LIMIT, ideal=False)
    paths.append(p)

  bg_paths = []
  for _ in range(BG_PATHS_TO_SIMULATE):
    p, _ = ice_engine.CreatePath(start_config, engine, all_obstacles, WALK_LIMIT, ideal=False)
    bg_paths.append(p)

  print 'Simulating paths...'
  successes = 0
  tles = 0
  iceds = 0
  for _ in xrange(PATHS_TO_SIMULATE_FOR_STATS):
    _, state = ice_engine.CreatePath(start_config, engine, all_obstacles, WALK_LIMIT, ideal=False)
    if state == ice_engine.SUCCESS:
      successes += 1
    elif state == ice_engine.TLE:
      tles += 1
    elif state == ice_engine.ICED:
      iceds += 1
    else:
      assert False
  print 'Paths simulation completed. Success: {0}, TLE: {1}, Iced: {2}'.format(
      successes, tles, iceds)

  end_time = time.clock()
  elapsed_time = end_time - start_time
  print '{0} seconds'.format(elapsed_time)

  def Callback(pos1, pos2, drawpath, drawideal):
    print pos1, pos2
    o = math.atan2(pos2[1] - pos1[1], pos2[0] - pos1[0]) % ANGLE_MOD
    p, _ = ice_engine.CreatePath((pos1, o), engine, all_obstacles, WALK_LIMIT, ideal=False)
    drawpath(p)
    p, _ = ice_engine.CreatePath((pos1, o), engine, all_obstacles, WALK_LIMIT, ideal=True)
    drawideal(p)

  draw.DrawSpace(start_config,
                 goal_config,
                 obstacles,
                 solution=ideal_path,
                 solutions=paths,
                 bg_solutions=bg_paths,
                 twopos_callback=Callback)
