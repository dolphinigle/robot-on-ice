import math

from shapely.geometry.linestring import LineString

from ice_robot import grid, markov
from ice_robot.markov import STEER_ANGLES
from nonholonomic_shortest_path import geom_util
from nonholonomic_shortest_path.geom_util import ANGLE_MOD, Arc, CLOCKWISE, \
  COUNTER_CLOCKWISE


WALK_LIMIT = 500

class SteeringGuide(object):
  '''Abstract class for implementing classes. This is pluggable to the AI'''
  '''configuration is ((x, y), angle)'''
  def __init__(self,
               goal_config,
               obstacles,
               max_steering_angle,
               vehicle_length,
               x_variance,
               y_variance,
               orientation_variance,
               average_path_length,
               axis_resolution,
               orientation_resolution,
               ):
    self.goal_config = goal_config
    self.max_steering_angle = max_steering_angle
    self.average_path_length = average_path_length
    self.vehicle_length = vehicle_length
    self.grid = grid.Grid(axis_resolution=axis_resolution,
                          orientation_resolution=orientation_resolution,
                          average_path_length=average_path_length,
                          vehicle_length=vehicle_length,
                          x_variance=x_variance,
                          y_variance=y_variance,
                          orientation_variance=orientation_variance,
                          obstacles=obstacles)


  def GetSteeringAngle(self, configuration):
    '''Retrieves the steering angle for the configuration.'''
    assert False, 'Implement'


class GreedySteeringGuide(SteeringGuide):
  def GetSteeringAngle(self, configuration):
    goal_angle = math.atan2(self.goal_config[0][1] - configuration[0][1],
                            self.goal_config[0][0] - configuration[0][0]) % ANGLE_MOD
    if geom_util.AngleAlmostEqual(goal_angle, configuration[1]):
      return 0.0
    leftdist = goal_angle - configuration[1]
    rightdist = configuration[1] - goal_angle
    if (leftdist % ANGLE_MOD < rightdist % ANGLE_MOD):
      return self.max_steering_angle
    else:
      return -self.max_steering_angle


class MarkovDecisionProcessGuide(SteeringGuide):
  def __init__(self,
               goal_config,
               obstacles,
               max_steering_angle,
               vehicle_length,
               x_variance,
               y_variance,
               orientation_variance,
               average_path_length,
               axis_resolution,
               orientation_resolution,
               iced_reward,
               goal_reward,
               discount,
               ):
    super(MarkovDecisionProcessGuide, self).__init__(goal_config,
                                                     obstacles,
                                                     max_steering_angle,
                                                     vehicle_length,
                                                     x_variance,
                                                     y_variance,
                                                     orientation_variance,
                                                     average_path_length,
                                                     axis_resolution,
                                                     orientation_resolution)
    self.state_to_action = markov.MarkovPrecomputeSteeringAngle(
        self.grid,
        goal_config,
        [mult * max_steering_angle for mult in STEER_ANGLES],
        discount,
        iced_reward,
        goal_reward,)
    print self.state_to_action

  def GetSteeringAngle(self, configuration):
    state = self.grid.ConfigurationToState(configuration)
    if state not in self.state_to_action:
      print 'WARNING: State {0} not found'.format(state)
      return 0.0
    return self.state_to_action[state]


def CreatePath(start_config, engine, ideal=True):
  current_path = []
  current_config = start_config
  walk_limit = WALK_LIMIT
  if ideal:
    sample_func = engine.grid.GetDestinationLocationAndOrientation
  else:
    sample_func = engine.grid.SampleMovement

  while (walk_limit and
         not markov.CloseToGoal(current_config, engine.goal_config, engine.grid) and
         not markov.OutsideBoundaries(current_config)):
    walk_limit -= 1
    steering_angle = engine.GetSteeringAngle(current_config)
    x, y, o = sample_func(current_config[0][0],
                          current_config[0][1],
                          current_config[1],
                          steering_angle,
                          engine.average_path_length)
    if geom_util.AngleAlmostEqual(steering_angle % ANGLE_MOD, 0.0):
      current_path.append(LineString([current_config[0], (x, y)]))
    else:
      radius = engine.vehicle_length / math.sin(steering_angle)
      antirangle = (math.pi / 2.0 - current_config[1]) % ANGLE_MOD
      dx = math.cos(antirangle) * radius
      dy = math.sin(antirangle) * radius
      circumrad = engine.average_path_length / abs(radius)
      if steering_angle > 0.0:
        direction = COUNTER_CLOCKWISE
        begin_radian = (current_config[1] - math.pi / 2.0) % ANGLE_MOD
        end_radian = (begin_radian + circumrad) % ANGLE_MOD
      else:
        direction = CLOCKWISE
        begin_radian = (current_config[1] + math.pi / 2.0) % ANGLE_MOD
        end_radian = (begin_radian - circumrad) % ANGLE_MOD
      current_path.append(Arc((current_config[0][0] - dx,
                               current_config[0][1] + dy),
                              abs(radius),
                              begin_radian,
                              end_radian,
                              direction,))
      end_pos = current_path[-1].AngleToPoint(end_radian)
      current_path.append(LineString([end_pos, (x, y)]))
    current_config = ((x, y), o)
  return current_path



