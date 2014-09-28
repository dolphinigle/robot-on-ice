'''
Created on Sep 27, 2014

@author: dolphinigle
'''
import math
import unittest

from shapely.geometry.linestring import LineString
from shapely.geometry.polygon import Polygon

from nonholonomic_shortest_path import geom_util
from nonholonomic_shortest_path.geom_util import Circle


def CCWize(points):
  if geom_util.IsCcw(LineString(points)):
    return points
  else:
    return points[::-1]



class ArcTest(unittest.TestCase):
  def setUp(self):
    self.cw = geom_util.Arc(center=(3, 4),
                            radius=5,
                            begin_radian=math.pi / 2.0,
                            end_radian = math.pi,
                            direction=geom_util.CLOCKWISE)
    self.ccw = geom_util.Arc(center=(3, 4),
                             radius=5,
                             begin_radian=math.pi / 2.0,
                             end_radian = math.pi,
                             direction=geom_util.COUNTER_CLOCKWISE)


  def tearDown(self):
    pass


  def testMidAngle(self):
    self.assertAlmostEquals(self.cw.MidAngle(),
                            math.pi * 7.0 / 4.0)
    self.assertAlmostEquals(self.ccw.MidAngle(),
                            math.pi * 3.0 / 4.0)
    self.assertAlmostEquals(self.ccw.MidAngle(0.25),
                            math.pi * 5.0 / 8.0)
    self.assertAlmostEquals(self.cw.MidAngle(2.0 / 3.0),
                            math.pi * 3.0 / 2.0)

  def testAngleToPoint(self):
    tests = [(math.asin(4.0 / 5.0), (6.0, 8.0)),
             (math.asin(3.0 / 5.0), (7.0, 7.0)),
             (math.asin(0), (8.0, 4.0)),
             (math.asin(1), (3.0, 9.0))]
    for test in tests:
      for arc in [self.cw, self.ccw]:
        point = arc.AngleToPoint(test[0])
        self.assertAlmostEquals(point[0],
                                test[1][0])
        self.assertAlmostEquals(point[1],
                                test[1][1])

  def testIsPointOnArcOrOnArcBoundary(self):
    self.assertTrue(self.cw.IsPointOnArcOrOnArcBoundary((3.0, 9.0)))
    self.assertTrue(self.ccw.IsPointOnArcOrOnArcBoundary((3.0, 9.0)))

    self.assertTrue(self.cw.IsPointOnArcOrOnArcBoundary((-2.0, 4.0)))
    self.assertTrue(self.ccw.IsPointOnArcOrOnArcBoundary((-2.0, 4.0)))

    tests_ccw = [(3 - 4, 4 + 3),
                 (3 - 3, 4 + 4)]
    tests_cw = [(3 + 4, 4 + 3),
                (3 + 3, 4 + 4),
                (3 + 4, 4 - 3),
                (3 + 3, 4 - 4),
                (3 - 4, 4 - 3),
                (3 - 3, 4 - 4),]
    for arc, trues, falses in zip([self.cw, self.ccw],
                                  [tests_cw, tests_ccw],
                                  [tests_ccw, tests_cw]):
      for point in trues:
        self.assertTrue(arc.IsPointOnArcOrOnArcBoundary(point))
      for point in falses:
        self.assertFalse(arc.IsPointOnArcOrOnArcBoundary(point))


  def testCircleLineSegmentIntersection(self):
    for arc in [self.cw, self.ccw]:
      points = geom_util.CircleLineSegmentIntersections(
          arc,
          LineString([(4, -2),
                      (-3, 5)]))
      self.assertEquals(2, len(points))
      self.assertAlmostEquals(points[0][0], 3)
      self.assertAlmostEquals(points[0][1], -1)
      self.assertAlmostEquals(points[1][0], -2)
      self.assertAlmostEquals(points[1][1], 4)

      points = geom_util.CircleLineSegmentIntersections(
          arc,
          LineString([(2, 0),
                      (-3, 5)]))
      self.assertEquals(1, len(points))
      self.assertAlmostEquals(points[0][0], -2)
      self.assertAlmostEquals(points[0][1], 4)


      points = geom_util.CircleLineSegmentIntersections(
          arc,
          LineString([(2, 0),
                      (-1, 3)]))
      self.assertEquals(0, len(points))


      points = geom_util.CircleLineSegmentIntersections(
          arc,
          LineString([(100, -98),
                      (101, -99)]))
      self.assertEquals(0, len(points))


      points = geom_util.CircleLineSegmentIntersections(
          arc,
          LineString([(100, 10004),
                      (101, 99)]))
      self.assertEquals(0, len(points))

      points = geom_util.CircleLineSegmentIntersections(
          arc,
          LineString([(6, 7),
                      (6, 9)]))
      self.assertEquals(1, len(points))


      points = geom_util.CircleLineSegmentIntersections(
          arc,
          LineString([(6, 7),
                      (6, 9)]))
      self.assertEquals(1, len(points))


  def testIsArcStrictIntersectPoly(self):
    for arc in [self.cw, self.ccw]:
      self.assertTrue(geom_util.IsArcStrictIntersectPoly(
          arc,
          Polygon(CCWize([(-5, -5), (100, -5), (100, 100), (-5, 100)]))))
      self.assertFalse(geom_util.IsArcStrictIntersectPoly(
          arc,
          Polygon(CCWize([(-5, -5), (-4, -5), (-4, -4), (-5, -5)]))))
      self.assertFalse(geom_util.IsArcStrictIntersectPoly(
          arc,
          Polygon(CCWize([(3, 3), (3, 4), (4, 4), (4, 3)]))))
      self.assertTrue(geom_util.IsArcStrictIntersectPoly(
          arc,
          Polygon(CCWize([(-100, 3), (100, 3), (100, 5), (-100, 5)]))))
      self.assertTrue(geom_util.IsArcStrictIntersectPoly(
          arc,
          Polygon(CCWize([(4, 3),
                          (4, 100),
                          (-100, 3)]))))
      self.assertFalse(geom_util.IsArcStrictIntersectPoly(
          arc,
          Polygon(CCWize([(3, -1),
                          (8, 4),
                          (3, 9),
                          (-2, 4)]))))
      self.assertFalse(geom_util.IsArcStrictIntersectPoly(
          arc,
          Polygon(CCWize([(0, -1),
                          (6, -1),
                          (6, -2),
                          (0, -2)]))))
      self.assertTrue(geom_util.IsArcStrictIntersectPoly(
          arc,
          Polygon(CCWize([(2, 3),
                          (-2, 9),
                          (4, 5),
                          (8, -1)]))))

    cw_true = [Polygon(CCWize([(6, 7),
                               (8, 7),
                               (8, 9),
                               (6, 9)])),
               Polygon(CCWize([(3, 4),
                               (-3, 4),
                               (2, 3),
                               (3, -2)])),
               Polygon(CCWize([(-2, 0),
                               (8, 0),
                               (8, -1),
                               (-2, -2)]))]
    ccw_true = [Polygon(CCWize([(-2, 6),
                                (-2, 9),
                                (1, 9)])),
                Polygon(CCWize([(-2, 5),
                                (2, 9),
                                (6, 0)]))]
    for arc, trues, falsies in zip([self.cw, self.ccw],
                                   [cw_true, ccw_true],
                                   [ccw_true, cw_true]):
      for poly in trues:
        self.assertTrue(geom_util.IsArcStrictIntersectPoly(arc, poly),
                        'Not True: Test {0}'.format(poly))
      for poly in falsies:
        self.assertFalse(geom_util.IsArcStrictIntersectPoly(arc, poly),
                         'Not False: Test {0}'.format(poly))

  def testIsCcw(self):
    self.assertTrue(geom_util.IsCcw(LineString([
        [0, 0],
        [1, 0],
        [0, 1]])))
    self.assertFalse(geom_util.IsCcw(LineString([
        [0, 0],
        [0, 1],
        [1, 0]])))


  def testGetTangentLine(self):
    circle1 = geom_util.Circle(center=(1, 2),
                               radius=1)
    circle2 = geom_util.Circle(center=(4, 2),
                               radius=1)

    angle1, angle2 = geom_util.GetTangentLine(
        circle1=circle1, is_circle1_ccw=True, circle2=circle2, is_circle2_ccw=True)
    self.assertAlmostEquals(
        angle1, math.pi * 3.0 / 2.0)
    self.assertAlmostEquals(
        angle2, math.pi * 3.0 / 2.0)

    angle1, angle2 = geom_util.GetTangentLine(
        circle1=circle1, is_circle1_ccw=False, circle2=circle2, is_circle2_ccw=False)
    self.assertAlmostEquals(
        angle1, math.pi * 1.0 / 2.0)
    self.assertAlmostEquals(
        angle2, math.pi * 1.0 / 2.0)

    circle1 = geom_util.Circle(center=(1, 3),
                               radius=1)
    circle2 = geom_util.Circle(center=(3, 1),
                               radius=1)

    angle1, angle2 = geom_util.GetTangentLine(
        circle1=circle1, is_circle1_ccw=True, circle2=circle2, is_circle2_ccw=False)
    self.assertAlmostEquals(
        angle1, math.pi * 3.0 / 2.0)
    self.assertAlmostEquals(
        angle2, math.pi * 1.0 / 2.0)

    angle1, angle2 = geom_util.GetTangentLine(
        circle1=circle1, is_circle1_ccw=False, circle2=circle2, is_circle2_ccw=True)
    self.assertAlmostEquals(
        angle1, 0.0)
    self.assertAlmostEquals(
        angle2, math.pi)

  def testCircleIntersects(self):
    self.assertFalse(geom_util.CircleIntersects(Circle(center=(2, 3),
                                                       radius=5),
                                                Circle(center=(14, 15),
                                                       radius=6)))
    self.assertTrue(geom_util.CircleIntersects(Circle(center=(2, 3),
                                                      radius=10),
                                               Circle(center=(14, 15),
                                                      radius=8)))
    self.assertFalse(geom_util.CircleIntersects(Circle(center=(0, 0),
                                                       radius=5),
                                                Circle(center=(0, 45),
                                                       radius=40)))
    self.assertTrue(geom_util.CircleIntersects(Circle(center=(0, 0),
                                                       radius=6),
                                                Circle(center=(0, 45),
                                                       radius=40)))


  def testIsAngleBetween(self):
    self.assertTrue(geom_util.IsAngleBetween(1.0, 0.0, 2.0))
    self.assertFalse(geom_util.IsAngleBetween(0.0, 0.0, 2.0))
    self.assertFalse(geom_util.IsAngleBetween(2.0, 0.0, 2.0))
    self.assertFalse(geom_util.IsAngleBetween(1.0, 2.0, 0.0))
    self.assertTrue(geom_util.IsAngleBetween(4.0, 2.0, 0.0))


if __name__ == "__main__":
  #import sys;sys.argv = ['', 'Test.testName']
  unittest.main()
