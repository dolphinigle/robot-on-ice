'''
Created on Nov 2, 2014

@author: dolphinigle
'''
from ice_robot import simulation, ice_engine


filename_dir = '../../inputs/'
simulation.Simulate(filename_dir + 'ice7.mp', ice_engine.MarkovDecisionProcessGuide)

