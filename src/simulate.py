## This File is where the simulations are actually run from
import sys
from Vector import Vec2d, Vec3d
from shapely.geometry import Point, Polygon, LineString
from UAV import UAV
from Environment import Environment
from Shark import Shark
from Cell_Plan import Cell_Plan
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
            (1000, 0),
            (1000, 1000),
            (0, 1000),
            (0, 0)
        ])

    env_config = {
        'uavs': [UAV(uav1_config), UAV(uav2_config)],
        'sharks': util.spawn_sharks(3, env_boundary),
        'boundary': env_boundary,
        'base_pos': control_pos,
        'plan': Cell_Plan({})
    }

    env = Environment(env_config)
    env.simulate()



if __name__ == "__main__":
    sim()
