## This File is where the simulations are actually run from
import sys
from Vector import Vec2d, Vec3d
from shapely.geometry import Point, Polygon, LineString
from UAV import UAV
from Environment import Environment
from Shark import Shark
from ChangeAlt_Plan import Change_Alt_Plan
from Basic_Plan import Basic_Plan
import util

def sim():
    control_pos = Vec3d()
    
    uav1_config = {
        'init_pos': control_pos,
        'init_energy': 100,
        'name': 'uav1'
    }
    uav2_config = {
        'init_pos': control_pos,
        'init_energy': 100,
        'name': 'uav2'
    }
    uav3_config = {
        'init_pos': control_pos,
        'init_energy': 100,
        'name': 'uav3'
    }

    env_boundary = Polygon([
            (0, 0), 
            (500, 0),
            (500, 500),
            (0, 500),
            (0, 0)
        ])

    #Environment configured with Changing Altitude Plan
    env_config = {
        'uavs': [UAV(uav1_config), UAV(uav2_config)],
        'sharks': util.spawn_sharks(3, env_boundary),
        'boundary': env_boundary,
        'base_pos': control_pos,
        'plan': Basic_Plan({}),

        'timestep': 0.1 # If you are plotting/creating a vid timestep should be < 1 / fps otherwise, 0.1 is okay.
    }

    env = Environment(env_config)
    # Video created if plotting=True, else no video
    env.simulate(plotting=True)


if __name__ == "__main__":
    sim()
