'''
Created on Sep 27, 2014

@author: dolphinigle
'''
from nonholonomic_shortest_path import draw, parse_input


if __name__ == '__main__':
  draw.DrawSpace(*parse_input.ReadInput('../../inputs/test_basic.mp'))
