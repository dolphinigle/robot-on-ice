'''
Created on Nov 2, 2014

@author: dolphinigle
'''

import hashlib
import math
import os
import pickle

from nonholonomic_shortest_path import geom_util
from nonholonomic_shortest_path.engine import Distance


STEER_ANGLES = [-1.0, 0.0, 1.0]
MAX_TURNING_ANGLE = math.pi / 4.0
VEHICLE_LENGTH = 0.1
AXIS_VARIANCE = 0.01
ORIENTATION_VARIANCE = 0.1
MAX_ITERS = 500
ITER_DIFF_LIMIT = 0.000001
AXIS_RESOLUTION = 20
ORIENTATION_RESOLUTION = 16
CACHE_DIR = '../../cache/'
ICED_REWARD = -10000
GOAL_REWARD = 100


def CloseToGoalConst(configuration, goal_configuration, g):
  if (Distance(configuration[0], goal_configuration[0]) <= 0.08 and
      geom_util.AngleShortestDiff(configuration[1], goal_configuration[1]) <= 0.4):
    return True
  return False


def CloseToGoal(configuration, goal_configuration, g):
  if (Distance(configuration[0], goal_configuration[0]) <= g.average_path_length and
      geom_util.AngleShortestDiff(configuration[1], goal_configuration[1]) <= g.orientation_tolerance):
    return True
  return False

def OutsideBoundaries(configuration):
  if configuration[0][0] < 0.0 or configuration[0][1] < 0.0 or configuration[0][0] > 1.0 or configuration[0][1] > 1.0:
    return True
  return False

def MarkovPrecomputeSteeringAngle(grid,
                                  goal_configuration,
                                  steering_angles,
                                  discount,
                                  iced_reward,
                                  goal_reward):
  '''steering_angles represent the possible steering angles. For the extreme case,
  use [-max, 0, max]'''
  # First, compute the goal states
  goal_states = set()
  for state in grid.StateGenerator():
    x, y, o = grid.StateToCoordinates(state)
    if CloseToGoal(((x, y), o), goal_configuration, grid):
      goal_states.add(state)

  # Now, generate the transitions
  print 'Generating transition probabilities...'

  hashhex = hashlib.sha224(pickle.dumps(grid) + str(steering_angles)).hexdigest()
  cachefile = CACHE_DIR + hashhex
  if os.path.isfile(cachefile):
    # Cache found
    print 'cache hit!'
    trans = pickle.load(open(cachefile, 'r'))
    print 'loaded'
  else:
    print 'cache miss...'
    trans = {}
    for state in grid.StateGenerator():
      print 'Generating for state {0}'.format(state)
      if not grid.IsBad(state):
        for action in steering_angles:
          trans[(state, action)] = grid.GenerateTransitionProbabilities(
              state, action)
          print len(trans[(state, action)])
    pickle.dump(trans, open(cachefile, 'w'))
    print 'Generated.'

  # Now initialize the "previous" values.
  prev_values = {}
  for state in grid.StateGenerator():
    if grid.IsBad(state):
      prev_values[state] = iced_reward
    elif state in goal_states:
      prev_values[state] = 0.0
    else:
      prev_values[state] = 0.0

  state_to_action = {}
  iters = MAX_ITERS
  print 'Performing iterations...'
  while iters:
    iters -= 1
    new_values = {}
    for state in grid.StateGenerator():
      if grid.IsBad(state):
        new_values[state] = iced_reward
      elif state in goal_states:
        new_values[state] = 0.0
      else:
        best_val = None
        for action in steering_angles:
          trans_prob_sum = 0.0
          val = 0.0
          for new_state, prob in trans[(state, action)]:
            trans_prob_sum += prob
            val += prob * discount * prev_values[new_state]
            assert not grid.IsBad(new_state)
            if new_state in goal_states:
              val += prob * goal_reward
          val += (1.0 - trans_prob_sum) * iced_reward
          if best_val is None or val > best_val:
            best_val = val
            state_to_action[state] = action
        new_values[state] = best_val
    maxdiff = 0.0
    for state in grid.StateGenerator():
      maxdiff = max(maxdiff, abs(new_values[state] - prev_values[state]))
    prev_values = new_values
    if maxdiff <= ITER_DIFF_LIMIT:
      print 'Iteration completed at {0}, maxdiff: {1}'.format(iters, maxdiff)
      break
    if iters % 20 == 0:
      print 'Iteration {0}, maxdiff: {1}'.format(MAX_ITERS - iters, maxdiff)

  return state_to_action

