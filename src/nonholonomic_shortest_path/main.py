'''
Created on Sep 27, 2014

@author: dolphinigle
'''
import time

from shapely.geometry.polygon import Polygon

from nonholonomic_shortest_path import draw, parse_input, engine

TIME_LIMIT = 30

if __name__ == '__main__':

  def RunAll(filename):
    level = 0
    while level < 8:
      print 'Level {0} ({1})'.format(level, filename)
      start_config, goal_config, obstacles = parse_input.ReadInput(filename)
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
                                          level=level,)
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

  filename_dir = '../../inputs/'

  filenames = ['test1.mp',
               'test2.mp',
               'test3.mp',
               'test4.mp',
               'test5.mp',
               'test6.mp']

  filenames = ['test4.mp']

  for filename in filenames:
    RunAll(filename_dir + filename)
  # RunAll(filename_dir + 'test5.mp')
  # import cProfile
  # cProfile.run("RunAll(filename_dir + 'test5.mp')")
