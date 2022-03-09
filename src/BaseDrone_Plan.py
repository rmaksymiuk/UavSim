# This module is to give the default definition of the 
# plan class, any other implementations of the plan class should
# extend and override this definition

import util
import numpy as np
from shapely.geometry import Point, Polygon
from Vector import Vec2d, Vec3d
from Path import Path
from Plan import Plan
import sys

class Cell_Plan(Plan):
    '''
    Plan class will get a configuration dictionary as input
    to set the variables of a particular plan
    '''
    def __init__(self, config):
        super(Cell_Plan, self).__init__(config)
        self.traverse_alt = util.set_or_default(config, 'traverse_alt', 30)
        self.speeds = util.set_or_default(config, 'uav_speeds', None)
        self.default_speed = 20


    '''
    This strategy follows from The Boustrophedon Cellular Decomposition.
    At the given traversal altitude, we will find the drone with the smallest
    frame size, and break the environment into a grid according to that frame size.

    After this, the grid will be broken up evenly into sectors and each UAV will plow
    through its sector. 

    UAVs will start and finish at the control station
    '''
    def get_initial_paths(self, env):
        # Find the minimum edge for any uav
        min_frame_edge = self.get_min_edge(env)

        # Set the speeds if they're empty
        if self.speeds is None:
            self.set_speeds(env)

        # Bounds for the enviornment polygon
        minx, miny, maxx, maxy = env.boundary.bounds

        # Draw a rectangle around the environment boundary
        x_starts = np.arange(start=minx, stop=maxx, step=min_frame_edge)
        y_starts = np.arange(start=miny, stop=maxy, step=min_frame_edge)

        # List the squares corresponding to the cells in the environment
        squares = []
        centroids = []
        for i in range(x_starts.shape[0] - 1):
            for j in range(y_starts.shape[0] - 1):
                if i % 2 == 1:
                    j = (y_starts.shape[0] - 1) - (j + 1)
                new_square = Polygon([
                    (x_starts[i], y_starts[j]),
                    (x_starts[i + 1], y_starts[j]),
                    (x_starts[i + 1], y_starts[j + 1]),
                    (x_starts[i], y_starts[j + 1]),
                    (x_starts[i], y_starts[j])])
                if new_square.intersects(env.boundary):
                    squares.append(new_square)
                    centroids.append(new_square.centroid)

        # Each drone will be assigned an even number of squares
        baseCounter = 0
        for drone in env.uavs:
            if drone.role == 'base':
                baseCounter = baseCounter + 1
        squares_per_drone = np.ceil(len(squares) / (len(env.uavs) - baseCounter))
        paths = [[] for uav in range(len(env.uavs) - baseCounter)]
        for i, centroid in enumerate(centroids):
            cur_drone = int(i // squares_per_drone)
            paths[cur_drone].append(Vec3d(centroid.x, centroid.y, self.traverse_alt))

       # Put the drone paths in a Path object 
        final_paths = []
        for i, path in enumerate(paths):
            path_with_control = path + [env.base_pos]
            print(path_with_control)
            path_speeds = [self.speeds[env.uavs[i].name] for p in path_with_control]
            final_paths.append((env.uavs[i], Path(path_with_control, path_speeds)))
        #print(final_paths)
        #print(type(final_paths))
        #for i in final_paths:
        #    print(i)
        #sys.exit()
        return final_paths

    '''
    In this plan, we will not change drone behavior based on the spotting of sharks
    '''

    def update_paths(self, env, spotted):
        free_drones = []
        paths = []
        final_paths = []
        # Adding the drones with none path to an array
        for possible_free_drone in env.uavs:
            if possible_free_drone.role == 'base':
                free_drones.append(possible_free_drone)
        print("UAVs: ", env.uavs)
        # print("Free Drones: ")
        # print(free_drones)
        for uav, pts in spotted:
            if uav.role == 'observer':
                for pt in pts:
                    # sets the amplitude of the drone to be z = 15
                    pt3d = pt.to_Vec3d(15)
                    paths.append(pt3d)
                    print("Pt: ", pt3d)
        if not free_drones:
            return []
        else:
            test = len(paths)
            print("Len :", test)
            speed_counter = len(paths)
            lst = [20] * (speed_counter + 1)
            final_paths.append((free_drones[0], Path(paths + [env.base_pos], lst)))
            #final_paths.append((free_drones[0], Path([env.base_pos], [20])))
            return final_paths
        # return []

    '''
    At the given traversal altitude, 
    '''
    def get_min_edge(self, env):
        frame_edges = []
        for uav in env.uavs:
            x_edge, y_edge = uav.get_frame_size(self.traverse_alt)
            frame_edges.append(x_edge)
            frame_edges.append(y_edge)
        min_frame_edge = min(frame_edges)
        return min_frame_edge
    
    '''
    Set the speed that each uav should travel at if there were no speeds given in the 
    constructor
    '''
    def set_speeds(self, env):
        speeds = {}
        for uav in env.uavs:
            speeds[uav.name] = self.default_speed
        self.speeds = speeds