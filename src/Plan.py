# This module is to give the default definition of the 
# plan class, any other implementations of the plan class should
# extend and override this definition

# Do Not Edit this Module! Instead, extend it in MyPlan.py

import util
import numpy as np
from shapely.geometry import Point, Polygon
from Vector import Vec2d, Vec3d
from Path import Path

class Plan:
    '''
    Plan class will get a configuration dictionary as input
    to set the variables of a particular plan
    '''
    def __init__(self, config):
        pass

    '''
    This method should be overridden by implementations of the plan class.
    It should reuturn a list of tuples in the following format:
        - [(uav1, Path1), ..., (uavn, Pathn)]
    '''
    def get_initial_paths(self, env):
        return [(uav, Path()) for uav in env.uavs]

    '''
    This method should be overridden by implementations of the plan class.
    It takes as input the environment, and a list of tuples containing uavs and
        the objects that they spotted.
        - [(uav1, [Vec2d_11, Vec2d_12, ...]), ..., (uavn, [Vec2d_n1, Vec2d_n2, ...])]
    It should reuturn a list of tuples in the following format:
        - [(uav1, Path1), ..., (uavn, Pathn)]
    '''
    def update_paths(self, env, spotted):
        return [(uav, Path()) for uav in env.uavs]

        
