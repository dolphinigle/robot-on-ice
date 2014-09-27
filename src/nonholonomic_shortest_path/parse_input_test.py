'''
Created on Sep 27, 2014

@author: dolphinigle
'''
import unittest

from nonholonomic_shortest_path import parse_input


class Test(unittest.TestCase):
  def testName(self):
    start_config, end_config, obstacles = parse_input.ReadInput('../../inputs/test_basic.mp')
    self.assertEquals(start_config, ((0.1, 0.1), 0.0))
    self.assertEquals(end_config, ((0.9, 0.9), 1.57))
    self.assertEquals(2, len(obstacles))
    self.assertAlmostEquals(0.08, obstacles[0].area)
    self.assertAlmostEquals(0.01, obstacles[1].area)


if __name__ == "__main__":
  #import sys;sys.argv = ['', 'Test.testName']
  unittest.main()