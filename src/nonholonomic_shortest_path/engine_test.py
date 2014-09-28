'''
Created on Sep 27, 2014

@author: dolphinigle
'''
import math
import unittest

from shapely.geometry.linestring import LineString

from nonholonomic_shortest_path import engine, parse_input


class EngineTest(unittest.TestCase):
  def setUp(self):
    self.start_config, self.goal_config, self.obstacles = parse_input.ReadInput('../../inputs/test_basic.mp')


  def testAdjCircles(self):
    circles = engine._AdjCircles(((2, 3), math.pi / 2.0), 4.0, level=0)
    self.assertEquals(2, len(circles))
    self.assertAlmostEquals(circles[0].center[0], -2.0)
    self.assertAlmostEquals(circles[0].center[1], 3.0)
    self.assertAlmostEquals(circles[0].radius, 4.0)
    self.assertAlmostEquals(circles[1].center[0], 6.0)
    self.assertAlmostEquals(circles[1].center[1], 3.0)
    self.assertAlmostEquals(circles[1].radius, 4.0)


  def testGenerateCircle(self):
    circle = engine._GenerateCircle(
        LineString([(1, 3), (1, 2), (2, 2)]),
        0.5,
        radius=math.sqrt(2))
    self.assertAlmostEquals(circle.center[0], 2)
    self.assertAlmostEquals(circle.center[1], 3)
    self.assertAlmostEquals(circle.radius, math.sqrt(2))


if __name__ == "__main__":
  #import sys;sys.argv = ['', 'Test.testName']
  unittest.main()