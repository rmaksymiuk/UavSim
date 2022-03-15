## This File is where the simulations are actually run from
# Here is an example simulate.py file that uses the BaseDrone Plan
    # You can find the BaseDrone Plan in ExPlans/BaseDrone_Plan.py
import sys
from Vector import Vec2d, Vec3d
from shapely.geometry import Point, Polygon, LineString
from UAV import UAV
from Environment import Environment
from Env_Object import Env_Object
from ChangeAlt_Plan import Change_Alt_Plan
from Basic_Plan import Basic_Plan
from BaseDrone_Plan import Cell_Plan
import util

def sim():
    control_pos = Vec3d()
    
    uav1_config = {
        'init_pos': control_pos,
        'init_energy': 100,
        'focus': 'shark',
        'name': 'uav1',
        'role': 'observer'
    }
    uav2_config = {
        'init_pos': control_pos,
        'init_energy': 100,
        'focus': 'shark',
        'name': 'uav2',
        'role': 'observer'
    }
    uav3_config = {
        'init_pos': control_pos,
        'init_energy': 100,
        'focus': 'shark',
        'name': 'uav3',
        'role': 'base'
    }

    env_boundary = Polygon([
            (0, 0), 
            (1000, 0),
            (1000, 1000),
            (0, 1000),
            (0, 0)
        ])
    
    sharks = util.spawn_objects(5, 'shark', env_boundary)
    fishes = util.spawn_objects(3, 'fish', env_boundary, non_focus_perf=0.7)
    rocks = util.spawn_objects(8, 'rock', env_boundary, non_focus_perf=0.1, speed=0)

    #Environment configured with Changing Altitude Plan
    env_config = {
        'uavs': [UAV(uav1_config), UAV(uav2_config), UAV(uav3_config)],
        'objects': sharks + fishes + rocks,
        'boundary': env_boundary,
        'base_pos': control_pos,
        'plan': Cell_Plan({}),

        'timestep': 0.01 # If you are plotting/creating a vid timestep should be < 1 / fps otherwise, 0.1 is okay.
    }

    env = Environment(env_config)
    # Video created if plotting=True, else no video
    env.simulate(plotting=False)


if __name__ == "__main__":
    sim()
