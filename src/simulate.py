## This File is where the simulations are actually run from
import sys
from Vector import Vec2d, Vec3d
from shapely.geometry import Point, Polygon, LineString
from UAV import UAV
from Environment import Environment
from Env_Object import Env_Object
from ChangeAlt_Plan import Change_Alt_Plan
from Basic_Plan import Basic_Plan
import util

def sim():
    control_pos = Vec3d()

    # Create your UAV instances
    # Normally you do this by creating a config dictionary
    #   for each UAV in the environment



    # Create your Object instances
    # These can typically be created with the spawn_objects function in
    #   the util.py file. 
    

    # Create your environment configuration
    # Create a configuration dictionary and give it as input
    #   to the Environment class

    # Call Env.simulate(plotting=True) to create a video
    # Call Env.simulate(plotting=False) to not create a video (faster)

if __name__ == "__main__":
    sim()
