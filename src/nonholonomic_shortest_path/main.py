'''
Created on Sep 27, 2014

@author: dolphinigle
'''
import time

from shapely.geometry.polygon import Polygon

from nonholonomic_shortest_path import draw, parse_input, engine

TIME_LIMIT = 60

if __name__ == '__main__':

  def RunAll():
    filename = '../../inputs/test5.mp'
    level = 0
    while True:
      print 'Level {0}'.format(level)
      start_config, goal_config, obstacles = parse_input.ReadInput(filename,
                                                                   enlargement=0.001)
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
      start_time = time.clock()
      path, length = engine.ConstructPath(start_config,
                                          goal_config,
                                          obstacles,
                                          additional_obstacles,
                                          level=level,
                                          can_change_direction=True)
      end_time = time.clock()
      elapsed_time = end_time - start_time
      print '{0} seconds: length is {1}'.format(elapsed_time, length)
      draw.DrawSpace(start_config,
                     goal_config,
                     obstacles,
                     solution=path)
      level += 1
      if elapsed_time > TIME_LIMIT:
        print 'Exceeded time limit. Stopping...',
        break

  RunAll()
  # import cProfile
  # cProfile.run('RunAll()')
