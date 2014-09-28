import math
import random
import sys

import pygame

from nonholonomic_shortest_path import geom_util


#import and init pygame
def DrawSpace(start_config,
              goal_config,
              obstacles,
              space_min=0.0,  # min x/y coordinate of the space
              space_max=1.0,  # max x/y coordinate of the space
              solution=None,
              ):

  screen_width = 640
  screen_height = 640
  start_config_color = (255, 0, 0)
  goal_config_color = (0, 255, 0)
  arc_color = (255, 255, 0)
  segment_color = (0, 255, 255)
  point_radius = 1

  def NormalizeDistance(distance):
    return int((distance - space_min) / (1.0 * space_max - space_min) * max(screen_width, screen_height))

  def NormalizePoint(point):
    return (int((point[0] - space_min) / (1.0 * space_max - space_min) * screen_width),
            screen_height - int((point[1] - space_min) / (1.0 * space_max - space_min) * screen_height),)

  def NormalizeAngle(angle):
    if angle >= math.pi:
      angle -= 2.0 * math.pi
    return angle

  pygame.init() 
  
  #create the screen
  window = pygame.display.set_mode((screen_width, screen_height)) 

  for obstacle in obstacles:
    pygame.draw.polygon(window,
                        (random.randint(0, 255),
                         random.randint(0, 255),
                         random.randint(0, 255)),
                        map(NormalizePoint, obstacle.exterior.coords))

  pygame.draw.circle(window,
                     start_config_color,
                     NormalizePoint(start_config[0]),
                     point_radius)
  pygame.draw.circle(window,
                     goal_config_color,
                     NormalizePoint(goal_config[0]),
                     point_radius)

  if solution:
    for item in solution:
      if isinstance(item, geom_util.Arc):
        center = NormalizePoint(item.center)
        radius = NormalizeDistance(item.radius)
        '''
        print 'Arc centered {0} from {1} to {2} ({3})'.format(item.center, item._begin_radian, item._end_radian, item.direction)
        print 'Points from {0} to {1}'.format(item.AngleToPoint(item._begin_radian),
                                              item.AngleToPoint(item._end_radian))
        '''
        begin_rad = item._begin_radian
        end_rad = item._end_radian
        if end_rad < begin_rad:
          curves = [[begin_rad, math.pi * 2], [0.0, end_rad]]
        else:
          curves = [[begin_rad, end_rad]]
        for brad, erad in curves:
          pygame.draw.arc(window,
                          arc_color,
                          (center[0] - radius,
                           center[1] - radius,
                           radius * 2,
                           radius * 2),
                          brad,
                          erad)
      else:
        # print 'Segment from {0} to {1}'.format(item.coords[0], item.coords[1])
        pygame.draw.line(window,
                         segment_color,
                         NormalizePoint(item.coords[0]),
                         NormalizePoint(item.coords[1]))

  #draw it to the screen
  pygame.display.flip() 
  
  #input handling (somewhat boilerplate code):
  quitted = False
  while not quitted: 
    for event in pygame.event.get(): 
      if event.type == pygame.QUIT: 
        pygame.quit()
        quitted = True
        break
      else: 
        pass
        # print event 

