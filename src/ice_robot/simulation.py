import time

from shapely import ops
from shapely.geometry.polygon import Polygon

from ice_robot import ice_engine
from ice_robot.markov import AXIS_RESOLUTION, ORIENTATION_RESOLUTION, \
  MAX_TURNING_ANGLE, VEHICLE_LENGTH, AXIS_VARIANCE, ORIENTATION_VARIANCE, \
  ICED_REWARD, GOAL_REWARD
from nonholonomic_shortest_path import parse_input, draw


PATHS_TO_SIMULATE = 5


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

  ideal_path = ice_engine.CreatePath(start_config, engine, ideal=True)
  paths = []
  for _ in range(PATHS_TO_SIMULATE):
    paths.append(ice_engine.CreatePath(start_config, engine, ideal=False))

  end_time = time.clock()
  elapsed_time = end_time - start_time
  print '{0} seconds'.format(elapsed_time)

  draw.DrawSpace(start_config,
                 goal_config,
                 obstacles,
                 solution=ideal_path,
                 solutions=paths)
