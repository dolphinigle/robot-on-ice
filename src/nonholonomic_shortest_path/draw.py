import math
import random

import pygame

from nonholonomic_shortest_path import geom_util


#import and init pygame
def DrawSpace(start_config,
              goal_config,
              obstacles,
              space_min=0.0,  # min x/y coordinate of the space
              space_max=1.0,  # max x/y coordinate of the space
              solution=None,
              solutions=None,
              bg_solutions=None,
              twopos_callback=None,
              ):

  screen_width = 640
  screen_height = 640
  start_config_color = (0, 255, 255)
  goal_config_color = (255, 255, 0)
  arc_color = (0, 0, 0)
  segment_color = (255, 0, 0)
  point_radius = int(0.08 * 640)

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

  def DrawBG():
    background = pygame.Surface(window.get_size()).convert()
    background.fill((255, 255, 255))
    window.blit(background, (0, 0))
  
    pygame.draw.circle(window,
                       goal_config_color,
                       NormalizePoint(goal_config[0]),
                       point_radius)
  

    obstacle_color = (51, 255, 204)
    for obstacle in obstacles:
      pygame.draw.polygon(window,
                          obstacle_color,
                          map(NormalizePoint, obstacle.exterior.coords))

    pygame.draw.circle(window,
                       start_config_color,
                       NormalizePoint(start_config[0]),
                       1)
  
  DrawBG()

  def DrawPath(path, arc_color, segment_color):
    for item in path:
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

  if bg_solutions:
    for path in bg_solutions:
      color = (random.randint(220, 255), random.randint(220, 255), random.randint(220, 255))
      DrawPath(path, color, color)
  if solutions:
    for path in solutions:
      color = (random.randint(0, 255), random.randint(0, 255), random.randint(0, 255))
      while min(color) > 100:
        color = (random.randint(0, 255), random.randint(0, 255), random.randint(0, 255))
      DrawPath(path, color, color)
  if solution:
    DrawPath(solution, arc_color, segment_color)

  pygame.draw.circle(window,
                     start_config_color,
                     NormalizePoint(start_config[0]),
                     1)

  #draw it to the screen
  pygame.display.flip() 
  
  #input handling (somewhat boilerplate code):
  quitted = False
  pos1 = None

  def NormalizeP(p):
    return (1.0 * p[0] / screen_height,
            1.0 * (screen_width - p[1]) / screen_width
            )

  while not quitted: 
    for event in pygame.event.get(): 
      if event.type == pygame.QUIT: 
        pygame.quit()
        quitted = True
        break
      elif event.type == pygame.KEYDOWN:
        if event.key == pygame.K_c:
          DrawBG()
          pygame.display.flip()
      elif event.type == pygame.MOUSEBUTTONUP:
        if twopos_callback:
          pos = pygame.mouse.get_pos()
          if pos1 is not None and pos1 != pos:
            color = (random.randint(0, 200), random.randint(0, 200), random.randint(0, 200))
            while max(color) < 150 or min(color) > 50:
              color = (random.randint(0, 200), random.randint(0, 200), random.randint(0, 200))
            def Callback(path):
              DrawPath(path, color, color)
            def CallbackIdeal(path):
              DrawPath(path, arc_color, segment_color)
            DrawBG()
            twopos_callback(NormalizeP(pos1),
                            NormalizeP(pos),
                            Callback,
                            CallbackIdeal)
            pygame.display.flip() 
      elif event.type == pygame.MOUSEBUTTONDOWN:
        pos1 = pygame.mouse.get_pos()
      else:
        pass

