import time

from shapely import ops
from shapely.geometry.polygon import Polygon

from ice_robot import ice_engine
from ice_robot.markov import STEP_LENGTH, AXIS_RESOLUTION, \
  ORIENTATION_RESOLUTION, MAX_TURNING_ANGLE, VEHICLE_LENGTH, AXIS_VARIANCE, \
  ORIENTATION_VARIANCE
from nonholonomic_shortest_path import parse_input, draw


def Simulate(filename, engine_class):
  start_config, goal_config, obstacles = parse_input.ReadInput(filename)
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

  engine = engine_class(
      goal_config=goal_config,
      obstacles=all_obstacles,
      max_steering_angle=MAX_TURNING_ANGLE,
      vehicle_length=VEHICLE_LENGTH,
      x_variance=AXIS_VARIANCE,
      y_variance=AXIS_VARIANCE,
      orientation_variance=ORIENTATION_VARIANCE,
      average_path_length=STEP_LENGTH,
      axis_resolution=AXIS_RESOLUTION,
      orientation_resolution=ORIENTATION_RESOLUTION,
  )

  path = ice_engine.CreatePath(start_config, engine, ideal=True)

  end_time = time.clock()
  elapsed_time = end_time - start_time
  print '{0} seconds'.format(elapsed_time)

  draw.DrawSpace(start_config,
                 goal_config,
                 obstacles,
                 solution=path)
