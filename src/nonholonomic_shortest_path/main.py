'''
Created on Sep 27, 2014

@author: dolphinigle
'''
from shapely.geometry.polygon import Polygon

from nonholonomic_shortest_path import draw, parse_input, engine


if __name__ == '__main__':

  def RunAll():
    start_config, goal_config, obstacles = parse_input.ReadInput('../../inputs/test2.mp')
    # start_config, goal_config, obstacles = parse_input.ReadInput('../../inputs/test1.mp')
    additional_obstacles = [
        Polygon([(0, 0),
                 (0, 2),
                 (-1, 2),
                 (-1, -1),
                 (2, -1),
                 (2, 2),
                 (1, 2),
                 (1, 0)]),
        Polygon([(2, 1),
                 (2, 2),
                 (-1, 2),
                 (-1, 1)])]
    engine.ConstructPath(start_config, goal_config, obstacles + additional_obstacles)
    '''
  draw.DrawSpace(start_config,
                 goal_config,
                 obstacles,
                 solution=path
                 )
                 '''

  import cProfile
  cProfile.run('RunAll()')
