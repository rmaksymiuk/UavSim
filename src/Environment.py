import util
from shapely.geometry import Point, Polygon
import numpy as np
from Cell_Plan import Cell_Plan
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
import os

class Environment:
    '''
    Initialize the state of the environment
    '''
    def __init__(self, config):
        # constants
        self.fig_out_dir = os.path.join('..', 'figs')
        self.vid_out_path = os.path.join('..', 'vids', 'sim.mp4')

        # configs
        self.uavs = util.set_or_err(config, 'uavs') # List[UAV]
        self.sharks = util.set_or_err(config, 'sharks') # List[Shark]
        self.boundary = util.set_or_err(config, 'boundary') # Polygon
        self.base_pos = util.set_or_err(config, 'base_pos') # Vec3d

        self.timestep = util.set_or_default(config, 'timestep', 0.01)
        self.plot_every = util.set_or_default(config, 'plotstep', 1)
        self.mission_time = util.set_or_default(config, 'mission_time', None)
        self.min_plot_z = util.set_or_default(config, "min_plot_z", -10)
        self.max_plot_z = util.set_or_default(config, "max_plot_z", 50)
        self.plot_height = util.set_or_default(config, 'plot_height', 50)
        self.plan = util.set_or_default(config, 'plan', Cell_Plan({})) # There 

        self.total_time = 0.0
        self.time_since_plot = 0.0

        self.assign_colors()
        self.set_paths(self.plan.get_initial_paths(self))


    '''
    Takes one step through the environment
    '''
    def step(self):
        # Step sharks through the simulation
        self.total_time += self.timestep
        self.time_since_plot += self.timestep
        for shark in self.sharks:
            shark.step(self.timestep)

        # List of tuples containing (uav, spotted shark location)
        uav_spotted = [] 
        for uav in self.uavs:
            spotted = uav.step(self.timestep, self.total_time, self.sharks)
            if len(spotted) > 0:
                uav_spotted.append((uav, spotted))
        if len(uav_spotted) > 0:
            self.set_paths(self.plan.update_paths(self, uav_spotted))

        
        # Check if we should plot results
        if self.time_since_plot > self.plot_every:
            self.time_since_plot = 0.
            self.plot_env()


    '''
    Simulates the environment until
        - Mission time exceeds max time limit
        - All drones have exhausted their paths
    '''
    def simulate(self):
        self.clear_fig_output()
        while True:
            self.step()
            end = True
            for uav in self.uavs:
                if uav.path:
                    print(uav.name, len(uav.path.points))
                    end = False
                    break
            if end:
                break
            if self.mission_time and (self.total_time > self.mission_time):
                break
        util.create_video(self.vid_out_path, self.fig_out_dir)

    '''
    Sets the paths of each uav in the environment according to the given tuples
    Tuples are of the form (uav, Path)
    '''
    def set_paths(self, paths):
        for uav, path in paths:
            if self.validate_path(path):
                uav.set_path(path)
            else:
                print('Invalid Path for UAV', uav.name)

    '''
    Ensures that each point in a given path is within the boundaries of the environment
    '''
    def validate_path(self, path):
        valid = True
        for point in path.points:
            if not self.boundary.intersects(point.to_point2d()):
                valid = False
                break
        return valid

    '''
    Returns the uav associated with the given name
    '''
    def get_uav(self, uav_name):
        for uav in self.uavs:
            if uav.name == uav_name:
                return uav


    '''
    Creates a 3d Plot of the environment
    '''
    def plot_env(self):
        fig = plt.figure()
        ax = fig.add_subplot(projection='3d')
        minx, miny, maxx, maxy = self.boundary.bounds
        ep = maxx / 100
        ax.set_xticks(np.linspace(minx, maxx, 10))
        ax.set_yticks(np.linspace(miny, maxy, 10))
        ax.set_zticks(np.linspace(self.min_plot_z, self.max_plot_z, 10))
        ax.set_xlim3d(left=minx + ep, right=maxx - ep)
        ax.set_ylim3d(bottom=miny + ep, top=maxy - ep)
        ax.set_zlim3d(bottom=self.min_plot_z + ep, top = self.max_plot_z - ep)
        ax.add_collection3d(Poly3DCollection(util.convert_3d(self.boundary), alpha=0.5, color='lightsteelblue'))
        ax.scatter([self.base_pos.x], [self.base_pos.y], [self.base_pos.z], marker='X', color='black')

        for uav in self.uavs:
            uav.plot_uav(ax, self.total_time)

        for shark in self.sharks:
            shark.plot_shark(ax)

        ax.legend()
        plt.savefig(os.path.join(self.fig_out_dir, '%08.03f' % self.total_time + '.png'))


    '''
    Assign a color to each UAV
    '''
    def assign_colors(self):
        colors = ['firebrick', 'lightsalmon', 'darkorange', 'gold', 'yellowgreen', 'darkseagreen',
                'lightseagreen', 'mediumblue', 'mediumpurple', 'violet', 'fuchsia', 'hotpink', 'crimson']
        for i, uav in enumerate(self.uavs):
            uav.set_color(colors[i % len(colors)])


    '''
    Removes all images from the figure output directory
    '''
    def clear_fig_output(self):
        for f in os.listdir(self.fig_out_dir):
            os.remove(os.path.join(self.fig_out_dir, f))




