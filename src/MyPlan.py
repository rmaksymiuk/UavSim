# Implement this module to provide the UAVs in the
# simulation initial paths, and allow them to update
# thier paths through updatepaths

import util
import numpy as np
from shapely.geometry import Point, Polygon
from Vector import Vec2d, Vec3d
from Path import Path
from Plan import Plan

class MyPlan(Plan):
    '''
    Plan class will get a configuration dictionary as input
    to set the variables of a particular plan
    '''
    def __init__(self, config):
        super(MyPlan, self).__init__(config)
        pass

    '''
    This method gets an instance of the environment class as input
    It should reuturn a list of tuples in the following format:
        - [(uav1, Path1), ..., (uavn, Pathn)]
    Each uav in the list will be updated to follow its respective path
    '''
    def get_initial_paths(self, env):
        return [(uav, Path()) for uav in env.uavs]

    '''
    This method takes as input the environment, and a list of tuples containing uavs and
        the 2d locations of objects that they spotted. These locations given as Vec2d objects.
        Vec2d is a class in Vector.py. 
        - [(uav1, [Vec2d_11, Vec2d_12, ...]), ..., (uavn, [Vec2d_n1, Vec2d_n2, ...])]
    It should reuturn a list of tuples in the following format:
        - [(uav1, Path1), ..., (uavn, Pathn)]
    '''
    def update_paths(self, env, spotted):
        return [(uav, Path()) for uav in env.uavs]
