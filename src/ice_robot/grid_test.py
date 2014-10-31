import math
import unittest

from ice_robot import grid
from nonholonomic_shortest_path import geom_util


class EngineTest(unittest.TestCase):
  def setUp(self):
    self.grid = grid.Grid(axis_resolution=5,
                          orientation_resolution=4,
                          average_path_length=0.2,
                          vehicle_length=0.01,
                          x_variance=0.05,
                          y_variance=0.05,
                          orientation_variance=0.1)


  def testStateToCoor(self):
    x, y, o = self.grid.StateToCoordinates((0, 0, 0))
    self.assertAlmostEquals(x, 0.1)
    self.assertAlmostEquals(y, 0.1)
    self.assertAlmostEquals(o, math.pi / 4.0)

    x, y, o = self.grid.StateToCoordinates((4, 4, 3))
    self.assertAlmostEquals(x, 0.9)
    self.assertAlmostEquals(y, 0.9)
    self.assertAlmostEquals(o, 7 * math.pi / 4.0)

    x, y, o = self.grid.StateToCoordinates((4, 0, 3))
    self.assertAlmostEquals(x, 0.9)
    self.assertAlmostEquals(y, 0.1)
    self.assertAlmostEquals(o, 7 * math.pi / 4.0)


  def testStateToRangeCoor(self):
    (xl, xh), (yl, yh), (ol, oh) = self.grid.StateToCoorRange((0, 1, 2))
    self.assertAlmostEquals(xl, 0.0)
    self.assertAlmostEquals(xh, 0.2)
    self.assertAlmostEquals(yl, 0.2)
    self.assertAlmostEquals(yh, 0.4)
    self.assertAlmostEquals(ol, math.pi)
    self.assertAlmostEquals(oh, math.pi * 3.0 / 2.0)

    (xl, xh), (yl, yh), (ol, oh) = self.grid.StateToCoorRange((4, 4, 3))
    self.assertAlmostEquals(xl, 0.8)
    self.assertAlmostEquals(xh, 1.0)
    self.assertAlmostEquals(yl, 0.8)
    self.assertAlmostEquals(yh, 1.0)
    self.assertAlmostEquals(ol, math.pi * 3.0 / 2.0)
    self.assertTrue(geom_util.AngleAlmostEqual(oh, math.pi * 2.0))


  def testGetDestinationLocationAndOrientation(self):
    x, y, o = self.grid.GetDestinationLocationAndOrientation(0.0, 1.0, math.pi / 2.0, 0.0, 0.2)
    self.assertAlmostEquals(x, 0.0)
    self.assertAlmostEquals(y, 1.0 + 0.2)
    self.assertAlmostEquals(o, math.pi / 2.0)

    x, y, o = self.grid.GetDestinationLocationAndOrientation(0.0, 1.0, math.pi / 2.0, math.pi, 0.2)
    self.assertAlmostEquals(x, 0.0)
    self.assertAlmostEquals(y, 1.0 - 0.2)
    self.assertAlmostEquals(o, math.pi / 2.0)

    g = grid.Grid(axis_resolution=5,
                  orientation_resolution=4,
                  average_path_length=math.pi / 2.0,
                  vehicle_length=1.0,
                  x_variance=0.05,
                  y_variance=0.05,
                  orientation_variance=0.1)
    x, y, o = g.GetDestinationLocationAndOrientation(0.0, 1.0, math.pi / 2.0, math.pi / 2.0, math.pi / 2.0)
    self.assertAlmostEquals(x, -1.0)
    self.assertAlmostEquals(y, 2.0)
    self.assertAlmostEquals(o, math.pi)

    g = grid.Grid(axis_resolution=5,
                  orientation_resolution=4,
                  average_path_length=math.pi / 2.0,
                  vehicle_length=1.0,
                  x_variance=0.05,
                  y_variance=0.05,
                  orientation_variance=0.1)
    x, y, o = g.GetDestinationLocationAndOrientation(0.0, 1.0, math.pi / 2.0, -math.pi / 2.0, math.pi / 2.0)
    self.assertAlmostEquals(x, 1.0)
    self.assertAlmostEquals(y, 2.0)
    self.assertTrue(geom_util.AngleAlmostEqual(o, 0.0))


  def testGenerateMarkovProbaiblities(self):
    g = grid.Grid(axis_resolution=5,
                  orientation_resolution=4,
                  average_path_length=0.4,
                  vehicle_length=0.4,
                  x_variance=0.05,
                  y_variance=0.05,
                  orientation_variance=0.1)

    tests = [[(2, 2, 0), math.pi / 4.0, 0.95, 1.0],
             [(2, 2, 0), -math.pi / 2.0, 0.95, 1.0],
             [(0, 0, 0), math.pi / 4.0, 0.9, 1.0],
             [(4, 4, 0), math.pi / 4.0, 0.0, 0.1],  # out of boundary
             ]
    for state, steer, minval, maxval in tests:
      states = g.GenerateTransitionProbabilities(state, steer)
      prob_sum = 0.0
      pos_states = set()
      for s, p in states:
        self.assertTrue(0 <= p <= 1.0)
        assert s not in pos_states
        pos_states.add(s)
        assert 0 <= s[0] < 5
        assert 0 <= s[1] < 5
        assert 0 <= s[2] < 4
        prob_sum += p
      self.assertGreater(prob_sum, minval)
      self.assertLessEqual(prob_sum, maxval)


  def testSampleMovement(self):
    g = grid.Grid(axis_resolution=5,
                  orientation_resolution=4,
                  average_path_length=0.4,
                  vehicle_length=0.4,
                  x_variance=0.01,
                  y_variance=0.01,
                  orientation_variance=0.05)

    for _ in range(5):
      x, y, o = g.SampleMovement(0.0, 0.0, math.pi / 2.0, 0.0, 0.5)
      self.assertTrue(-0.1 <= x <= 0.1, x)
      self.assertTrue(0.4 <= y <= 0.6, y)
      self.assertTrue(math.pi / 2.0 - 0.5 <= o <= math.pi / 2.0 + 0.5, o)

