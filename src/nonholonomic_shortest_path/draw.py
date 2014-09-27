import sys
#import and init pygame
import pygame
import random

def DrawSpace(
              start_config,
              goal_config,
              obstacles,
              space_min=0.0,  # min x/y coordinate of the space
              space_max=1.0,  # max x/y coordinate of the space
              ):

  screen_width = 480
  screen_height = 480
  start_config_color = (255, 0, 0)
  goal_config_color = (0, 255, 0)
  point_radius = 1

  def NormalizePoint(point):
    return ((point.x - space_min) / (1.0 * space_max - space_min) * screen_width,
            (point.y - space_min) / (1.0 * space_max - space_min) * screen_height,)

  pygame.init() 
  
  #create the screen
  window = pygame.display.set_mode((screen_width, screen_height)) 

  for obstacle in obstacles:
    pygame.draw.polygon(window,
                        (random.randint(0, 255),
                         random.randint(0, 255),
                         random.randint(0, 255)),
                        map(NormalizePoint, obstacle.vertices))

  pygame.draw.circle(window,
                     start_config_color,
                     NormalizePoint(start_config[0]),
                     point_radius)
  pygame.draw.circle(window,
                     goal_config_color,
                     NormalizePoint(goal_config[0]),
                     point_radius)
  
  #draw it to the screen
  pygame.display.flip() 
  
  #input handling (somewhat boilerplate code):
  while True: 
    for event in pygame.event.get(): 
      if event.type == pygame.QUIT: 
        sys.exit(0) 
      else: 
        pass
        # print event 

