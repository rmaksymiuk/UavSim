## This File is where the simulations are actually run from
# Here is an example simulate.py file that uses the BaseDrone Plan
    # You can find the BaseDrone Plan in ExPlans/BaseDrone_Plan.py
import sys
from Vector import Vec2d, Vec3d
from shapely.geometry import Point, Polygon, LineString
from UAV import UAV
from Environment import Environment
from Env_Object import Env_Object
from Basic_Plan import Basic_Plan
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

    wind_bounds = [
            Polygon([
                (0, 0),
                (320, 500),
                (750, 750),
                (1000, 0),
                (0, 0)]),
            Polygon([
                (0, 0),
                (0, 1000),
                (1000, 1000),
                (1000, 0),
                (750, 750),
                (320, 500),
                (0, 0)])
            ]

    wind_bounds_small = [
            Polygon([
                (0, 0),
                (160, 250),
                (375, 375),
                (500, 0),
                (0, 0)]),
            Polygon([
                (0, 0),
                (0, 500),
                (500, 500),
                (500, 0),
                (375, 375),
                (160, 250),
                (0, 0)])
            ]
    wind_vels = [Vec3d(10, 10, 0), Vec3d(-10, 0, 0)]


    env_boundary = Polygon([
            (0, 0), 
            (1000, 0),
            (1000, 1000),
            (0, 1000),
            (0, 0)
        ])

    env_boundary_small = Polygon([
            (0, 0), 
            (500, 0),
            (500, 500),
            (0, 500),
            (0, 0)
        ])
    
    sharks = util.spawn_objects(5, 'shark', env_boundary)
    fishes = util.spawn_objects(3, 'fish', env_boundary, non_focus_perf=0.7)

    #Environment configured with Changing Altitude Plan
    env_config = {
        'uavs': [UAV(uav1_config), UAV(uav2_config), UAV(uav3_config)],
        'objects': sharks + fishes,
        'boundary': env_boundary,
        'base_pos': control_pos,
        'plan': Basic_Plan({}),
        'wind_regions': (wind_bounds, wind_vels),
        'timestep': 0.1 
    }

    env = Environment(env_config)
    # Video created if plotting=True, else no video
    env.simulate(plotting=True)


if __name__ == "__main__":
    sim()
