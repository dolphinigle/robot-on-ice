
import itertools
import math
import sys

import numpy.random
from scipy import stats
from shapely.geometry.polygon import Polygon

from nonholonomic_shortest_path import geom_util
from nonholonomic_shortest_path.geom_util import ANGLE_MOD


def circle_around(x, y):
    yield 0, (x, y)
    r = 1
    i, j = x-1, y-1
    while True:
        while i < x+r:
            i += 1
            yield r, (i, j)
        while j < y+r:
            j += 1
            yield r, (i, j)
        while i > x-r:
            i -= 1
            yield r, (i, j)
        while j > y-r:
            j -= 1
            yield r, (i, j)
        r += 1
        j -= 1
        yield r, (i, j)


class Grid(object):
  '''Represents the discretized space of the configuration space of the robot.
  Each state is a tuple (i, j, k), all 0-indexed.
  and orientation, respectively.
  '''

  def __init__(self,
               axis_resolution,
               orientation_resolution,
               average_path_length,
               vehicle_length,
               x_variance,
               y_variance,
               orientation_variance,
               obstacles=None,
               ):
    '''
    resolution is integers representing the cardinality of the dimensions.
    For exaple, axis_resolution=4 would create a 4x4 grid.

    average_path_length is the average length of a path taken for the transition
    probability.
    '''
    self.axis_resolution = axis_resolution
    self.orientation_resolution = orientation_resolution
    self.average_path_length = average_path_length
    self.vehicle_length = vehicle_length
    self.x_variance = x_variance
    self.y_variance = y_variance
    self.orientation_variance = orientation_variance

    is_bad = [[False] * axis_resolution for _ in range(axis_resolution)]
    if obstacles:
      for r, c in itertools.product(range(axis_resolution), range(axis_resolution)):
        (x1, x2), (y1, y2), (_, _) = self.StateToCoorRange((r, c, 0))
  
        # CCW
        square = Polygon([(x2, y2),
                          (x1, y2),
                          (x1, y1),
                          (x2, y1)])
        if square.intersects(obstacles) or obstacles.contains(square):
          is_bad[r][c] = True

    self.is_bad = is_bad

  def StateToCoordinates(self, state):
    '''Converts state to real coordinates and orientation. The coordinates are between 0.0 and 1.0,
    while the orientation is between 0 and 2 pi'''
    assert 0 <= state[0] < self.axis_resolution
    assert 0 <= state[1] < self.axis_resolution
    assert 0 <= state[2] < self.orientation_resolution

    grid_size = 1.0 / self.axis_resolution
    angle_size = ANGLE_MOD / self.orientation_resolution
    return (grid_size * (0.5 + state[0]),
            grid_size * (0.5 + state[1]),
            angle_size * (0.5 + state[2]))


  def ConfigurationToState(self, configuration):
    grid_size = 1.0 / self.axis_resolution
    base_r = int(math.floor(configuration[0][0] / grid_size))
    base_c = int(math.floor(configuration[0][1] / grid_size))
    def Normalize(v):
      if v < 0:
        v = 0
      if v >= self.axis_resolution:
        v = self.axis_resolution-1
      return v
    r = Normalize(base_r)
    c = Normalize(base_c)
    for io in range(self.orientation_resolution):
      _, _, (o1, o2) = self.StateToCoorRange((0, 0, io))
      if geom_util.IsAngleBetween(configuration[1], o1, o2):
        return (r, c, io)

    # Already in ice/goal
    return 0.0


  def StateToCoorRange(self, state):
    '''Given a state, gives the associate bounding box of the cell and the orientation.
    Returns it as a tuple ((x_low, x_hi), (y_low, y_hi), (o_low, o_hi))'''
    xmid, ymid, omid = self.StateToCoordinates(state)
    grid_size = 1.0 / self.axis_resolution
    angle_size = ANGLE_MOD / self.orientation_resolution
    return ((xmid - grid_size * 0.5, xmid + grid_size * 0.5),
            (ymid - grid_size * 0.5, ymid + grid_size * 0.5),
            ((omid - angle_size * 0.5) % ANGLE_MOD, (omid + angle_size * 0.5) % ANGLE_MOD))


  def GetDestinationLocationAndOrientation(self, x, y, orientation, steering_angle, distance):
    if geom_util.AngleAlmostEqual(steering_angle, 0.0):
      nx = x + math.cos(orientation) * distance
      ny = y + math.sin(orientation) * distance
      no = orientation
    elif geom_util.AngleAlmostEqual(steering_angle, math.pi):
      nx = x - math.cos(orientation) * distance
      ny = y - math.sin(orientation) * distance
      no = orientation
    else:
      radius = self.vehicle_length / math.sin(abs(steering_angle))
      circ_angle = distance / radius
      dyc = math.sin(circ_angle) * radius
      dy = math.sin(orientation) * dyc
      dx = math.cos(orientation) * dyc
      dxc = radius - math.cos(circ_angle) * radius
      dxc_angle = (orientation + 0.5 * math.pi) % ANGLE_MOD
      no = orientation + circ_angle
      if steering_angle < 0.0:
        dxc_angle = (orientation - 0.5 * math.pi) % ANGLE_MOD
        no = orientation - circ_angle
      dy += math.sin(dxc_angle) * dxc
      dx += math.cos(dxc_angle) * dxc
      nx = x + dx
      ny = y + dy

    return nx, ny, (no % ANGLE_MOD)


  def GenerateTransitionProbabilities(self, state, steering_angle):
    '''
    Generates all pairs of (state, probability) such that when starting in state
    "state" and driving on "steering angle", the probabilities of arriving in
    each of the states. This probability may not sum to 1.0 -- the lost probability
    is the robot falling into ice while following this route.

    The resulting location will follow a Gaussian distribution with standard
    deviation proportional to the actual distance travelled (as this approximates
    the mean of sum of independent random trials in the limit).

    The result is a list of tuples [(state, prob), ...]
    '''
    x, y, orientation = self.StateToCoordinates(state)
    nx, ny, no = self.GetDestinationLocationAndOrientation(x,
                                                           y,
                                                           orientation,
                                                           steering_angle,
                                                           self.average_path_length)
    x_dist = stats.norm(loc=nx,
                        scale=self.x_variance)
    y_dist = stats.norm(loc=ny,
                        scale=self.y_variance)
    o_dist = stats.norm(loc=no,
                        scale=self.orientation_variance)
    eps = sys.float_info.epsilon

    res = []
    bx, by, _ = self.ConfigurationToState(((nx, ny), no))
    prev_r = 0
    present = True
    for r, (ix, iy) in circle_around(bx, by):
      if r != prev_r and not present:
        break
      if r != prev_r:
        prev_r = r
        present = False
      if ix < 0 or iy < 0 or ix >= self.axis_resolution or iy >= self.axis_resolution:
        continue
      for io in range(self.orientation_resolution):
        (xlow, xhi), (ylow, yhi), (olow, ohi) = self.StateToCoorRange((ix, iy, io))
        prob = (x_dist.cdf(xhi) - x_dist.cdf(xlow)) * (y_dist.cdf(yhi) - y_dist.cdf(ylow))
        def TryBoth(lo, ho):
          milo = lo
          miho = ho
          if milo < no:
            milo += ANGLE_MOD
          if miho < no:
            miho += ANGLE_MOD
          nilo = lo
          niho = ho
          if nilo > no:
            nilo -= ANGLE_MOD
          if niho > no:
            niho -= ANGLE_MOD
          assert milo <= miho
          assert nilo <= niho
          return max(o_dist.cdf(miho) - o_dist.cdf(milo),
                     o_dist.cdf(niho) - o_dist.cdf(nilo))
  
        if geom_util.IsAngleBetween(no, olow, ohi):
          if olow > no:
            olow -= ANGLE_MOD
          if ohi < no:
            ohi += ANGLE_MOD
          assert olow <= ohi
          prob *= (o_dist.cdf(ohi) - o_dist.cdf(olow))
        elif geom_util.IsAngleBetween((no + math.pi) % ANGLE_MOD, olow, ohi):
          # Mirrored
          mid = (no + math.pi) % ANGLE_MOD
          prob *= TryBoth(olow, mid) + TryBoth(mid, ohi)
        else:
          prob *= TryBoth(olow, ohi)
  
        if prob < eps:
          continue
        if self.is_bad[ix][iy]:
          continue
        present = True
        res.append(((ix, iy, io), prob))
    return res


  def SampleMovement(self, x, y, orientation, steering_angle, distance):
    '''Samples a movement from (x, y, orientation). It is possible for this to
    end up not within the boundary of the field. The movement length will be
    approximately equal to the movement length.'''
    nx, ny, no = self.GetDestinationLocationAndOrientation(x,
                                                           y,
                                                           orientation,
                                                           steering_angle,
                                                           distance)
    return (numpy.random.normal(nx, self.x_variance, 1)[0],
            numpy.random.normal(ny, self.y_variance, 1)[0],
            numpy.random.normal(no, self.orientation_variance, 1)[0])


  def StateGenerator(self):
    for r, c, o in itertools.product(range(self.axis_resolution),
                                     range(self.axis_resolution),
                                     range(self.orientation_resolution)):
      yield r, c, o

  def IsBad(self, state):
    '''Is this cell obstructs obstacle?'''
    return self.is_bad[state[0]][state[1]]


