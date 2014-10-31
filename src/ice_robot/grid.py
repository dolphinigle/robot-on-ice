
import itertools
import math
import sys

from scipy import stats

from nonholonomic_shortest_path import geom_util
from nonholonomic_shortest_path.geom_util import ANGLE_MOD


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


  def StateToCoorRange(self, state):
    '''Given a state, gives the associate bounding box of the cell and the orientation.
    Returns it as a tuple ((x_low, x_hi), (y_low, y_hi), (o_low, o_hi))'''
    xmid, ymid, omid = self.StateToCoordinates(state)
    grid_size = 1.0 / self.axis_resolution
    angle_size = ANGLE_MOD / self.orientation_resolution
    return ((xmid - grid_size * 0.5, xmid + grid_size * 0.5),
            (ymid - grid_size * 0.5, ymid + grid_size * 0.5),
            ((omid - angle_size * 0.5) % ANGLE_MOD, (omid + angle_size * 0.5) % ANGLE_MOD))


  def GetDestinationLocationAndOrientation(self, x, y, orientation, steering_angle):
    if geom_util.AngleAlmostEqual(steering_angle, 0.0):
      nx = x + math.cos(orientation) * self.average_path_length
      ny = y + math.sin(orientation) * self.average_path_length
      no = orientation
    elif geom_util.AngleAlmostEqual(steering_angle, math.pi):
      nx = x - math.cos(orientation) * self.average_path_length
      ny = y - math.sin(orientation) * self.average_path_length
      no = orientation
    else:
      radius = self.vehicle_length / math.sin(abs(steering_angle))
      circ_angle = self.average_path_length / radius
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
                                                           steering_angle)
    x_dist = stats.norm(loc=nx,
                        scale=self.x_variance)
    y_dist = stats.norm(loc=ny,
                        scale=self.y_variance)
    o_dist = stats.norm(loc=no,
                        scale=self.orientation_variance)
    eps = sys.float_info.epsilon

    res = []
    for ix, iy, io in itertools.product(range(self.axis_resolution),
                                        range(self.axis_resolution),
                                        range(self.orientation_resolution)):
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
      res.append(((ix, iy, io), prob))
    return res



