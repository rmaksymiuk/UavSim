import util
from shapely.geometry import Point, Polygon
import numpy as np
import pandas as pd
from Basic_Plan import Basic_Plan
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
from shapely.ops import unary_union
from Vector import Vec3d
from Wind import Wind
import os
import shutil
import datetime

class Environment:
    '''
    Initialize the state of the environment
    '''
    def __init__(self, config):
        # constants
        self.fig_out_dir = os.path.join('..', 'figs')

        self.vid_out_dir = os.path.join('..', 'results')
        self.vid_out_path = os.path.join('..', 'results', 'sim.mp4')
        self.stats_out_path = os.path.join('..', 'results', 'stats.csv')


        # configs
        self.uavs = util.set_or_err(config, 'uavs') # List[UAV]
        self.objects = util.set_or_err(config, 'objects') # List[Env_Object]
        self.boundary = util.set_or_err(config, 'boundary') # Polygon
        self.base_pos = util.set_or_err(config, 'base_pos') # Vec3d

        self.wind_regions = util.set_or_default(config, 'wind_regions', None)
        self.timestep = util.set_or_default(config, 'timestep', 0.01)
        self.plot_every = util.set_or_default(config, 'plotstep', 0.5)
        self.mission_time = util.set_or_default(config, 'mission_time', None)
        self.min_plot_z = util.set_or_default(config, "min_plot_z", -10)
        self.max_plot_z = util.set_or_default(config, "max_plot_z", 50)
        self.plot_height = util.set_or_default(config, 'plot_height', 50)
        self.plan = util.set_or_default(config, 'plan', Basic_Plan({})) # There 
        self.env_force = util.set_or_default(config, 'env_force', Vec3d(0, 0, -9.81))

        self.total_time = 0.0
        self.time_since_plot = 0.0

        self.set_winds()
        self.assign_colors()
        self.assign_markers()
        self.set_env_force() #Let all the UAVs know the environmental force
        self.set_paths(self.plan.get_initial_paths(self))


    '''
    Takes one step through the environment
    If plotting is true, plot the progress of the environment as you step through
    it, otherwise, skip plotting
    '''
    def step(self, plotting):
        # Step Wind objects through simulation (for plotting)
        for wind in self.winds:
            wind.step(self.timestep)

        # Step env_objects through the simulation
        self.total_time += self.timestep
        self.time_since_plot += self.timestep
        for env_object in self.objects:
            env_object.step(self.timestep)

        # List of tuples containing (uav, spotted object location)
        uav_spotted = [] 
        for uav in self.uavs:
            spotted = uav.step(
                    self.timestep, 
                    self.total_time, 
                    self.objects, 
                    self.base_pos,
                    self.winds)
            if len(spotted) > 0:
                uav_spotted.append((uav, spotted))
        if len(uav_spotted) > 0:
            self.set_paths(self.plan.update_paths(self, uav_spotted))

        
        # Check if we should plot results
        if plotting and (self.time_since_plot > self.plot_every):
            self.time_since_plot = 0.
            self.plot_env()


    '''
    Simulates the environment until
        - Mission time exceeds max time limit
        - All drones have exhausted their paths
    If plotting is true, the state of the environment will be plotted 
    as the simulation proceeds, and a video of the simulation will be
    comipled at the end. Otherwise, only stats are saved from the simulation
    '''
    def simulate(self, plotting=True):
        start = datetime.datetime.now()
        if plotting:
            self.create_dirs()
            self.clear_fig_output()
        while True:
            self.step(plotting)
            end = True
            for uav in self.uavs:
                if uav.path:
                    end = False
                    break
            if end:
                break
            if self.mission_time and (self.total_time > self.mission_time):
                break
        if plotting:
            util.create_video(self.vid_out_path, self.fig_out_dir)
            self.clean_up()
        self.get_stats()
        end = datetime.datetime.now()
        clock_time = (end - start).total_seconds()
        sps = self.total_time / clock_time / self.timestep
        print('Clock Time:', clock_time, 'seconds')
        print('Simulation Time:', self.total_time, 'seconds')
        print('Steps Per Second:', sps, 'steps')

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

        for env_object in self.objects:
            env_object.plot_object(ax)

        for wind in self.winds:
            wind.plot(ax)

        ax.legend()
        plt.savefig(os.path.join(self.fig_out_dir, '%08.03f' % self.total_time + '.png'))
        plt.close()

    '''
    Create a dataframe with the statistics for this simulation and output it to
    the result directory
    '''
    def get_stats(self):
        # Get type df: Contains count of each type of object: Type, Count
        count_df = self.get_type_df()
        # Build UAV df
        # Name, Focus, TP, FP, FN, Energy Used, Area Covered
        uav_dfs = []
        for uav in self.uavs:
            uav_dfs.append(uav.get_stats())
        uav_df = pd.DataFrame(uav_dfs)
        uav_df = pd.merge(uav_df, count_df, how='inner', left_on='Focus', right_on='Type')

        # Build Object Df
        # Object, UAV
        object_dfs = []
        for env_object in self.objects:
            one_object_df = env_object.get_stats()
            if len(one_object_df) > 0:
                # only add the object df if the object was seen
                object_dfs.append(one_object_df)
        total_objects_spotted = len(object_dfs)
        if len(object_dfs) > 0:
            object_df = pd.concat(object_dfs, axis=0)
            objects_by_uav = object_df['UAV'].value_counts()
            objects_by_uav.name = 'Num_Objects'
            uav_df = pd.merge(uav_df, objects_by_uav, left_on='Name', right_index=True, how='left')
            uav_df.loc[pd.isna(uav_df['Num_Objects']), 'Num_Objects'] = 0
        else:
            uav_df['Num_Objects'] = 0
            

        # Calculate totals
        total_values = uav_df.aggregate({
            'TP': 'sum',
            'FP': 'sum',
            'FN': 'sum',
            'Energy_Used': 'sum',
            'Area_Covered': util.list_union})
        total_values['Name'] = 'Total'
        total_values['Num_Objects'] = total_objects_spotted
        total_values['Count'] = count_df.loc[count_df['Type'].isin(list(uav_df['Focus'].unique())), 'Count'].sum()

        # Merge total with UAVs
        all_df = pd.concat([uav_df, total_values.to_frame().transpose()], axis=0).reset_index()

        # Final Calculations
        all_df.loc[(all_df['TP'] == 0) & (all_df['FN'] == 0), 'FN'] = np.nan # Handle div by 0 if no TP or FN
        all_df.loc[(all_df['TP'] == 0) & (all_df['FP'] == 0), 'FP'] = np.nan # Handle div by 0 if no TP or FP
        all_df['Precision'] = all_df['TP'] / (all_df['TP'] + all_df['FP'])
        all_df['Recall'] = all_df['TP'] / (all_df['TP'] + all_df['FN'])
        all_df['F1'] = 2 * all_df['Precision'] * all_df['Recall'] / (all_df['Precision'] + all_df['Recall'])
        all_df['PCT_Covered'] = all_df['Area_Covered'].apply(lambda x: x.intersection(self.boundary).area) / self.boundary.area
        all_df['PCT_Spotted'] = all_df['Num_Objects'] / all_df['Count']

        interesting_cols = all_df[['Name', 'Precision', 'Recall', 'F1', 'Energy_Used', 'PCT_Covered', 'PCT_Spotted']]
        print(interesting_cols)
        interesting_cols.to_csv(self.stats_out_path, index=False)



    '''
    Create Wind objects for the given wind regions
    '''
    def set_winds(self):
        self.winds = []
        if self.wind_regions is None:
            return
        if len(self.wind_regions) != 2:
            print('Illegal Wind Specification. Proceeding Without wind')
            return
        boundaries = self.wind_regions[0]
        velocities = self.wind_regions[1]
        if not len(boundaries) == len(velocities):
            print('Illegal Wind Specification. Proceeding Without wind')
            return
        first = True
        for i, bound in enumerate(boundaries):
            self.winds.append(Wind(bound, velocities[i], self.boundary, first))
            first=False


    '''
    Tell all the UAVs what the environmental force is
    '''
    def set_env_force(self):
        for uav in self.uavs:
            uav.set_env_force(self.env_force)

    '''
    Assign a color to each UAV
    '''
    def assign_colors(self):
        colors = ['firebrick', 'lightsalmon', 'darkorange', 'gold', 'yellowgreen', 'darkseagreen',
                'lightseagreen', 'mediumblue', 'mediumpurple', 'violet', 'fuchsia', 'hotpink', 'crimson']
        for i, uav in enumerate(self.uavs):
            uav.set_color(colors[i % len(colors)])

    '''
    Assign a marker to each object. Also tell each object if they should include a legend.
    Only the first object of each class should include a legend
    '''
    def assign_markers(self):
        types = {}
        markers = ['2', 's', 'P', '*', 'D']
        cur_marker = 0
        for o in self.objects:
            if o.type not in types:
                types[o.type] = markers[cur_marker % len(markers)]
                cur_marker += 1
                o.assign_marker(types[o.type], first=True)
            else:
                o.assign_marker(types[o.type], first=False)

    '''
    Creates a dataframe containing the type of an object and the count of each
    type of object
    '''
    def get_type_df(self):
        types = {}
        for o in self.objects:
            if o.type not in types:
                types[o.type] = 0
            types[o.type] += 1
        return pd.DataFrame(types.items(), columns=['Type', 'Count'])

    '''
    Removes all images from the figure output directory
    '''
    def clear_fig_output(self):
        for f in os.listdir(self.fig_out_dir):
            os.remove(os.path.join(self.fig_out_dir, f))


    '''
    Create output directories if they don't exists
    '''
    def create_dirs(self):
        # make the figure output directory if it doesn't exist
        if not os.path.exists(self.fig_out_dir):
            os.mkdir(self.fig_out_dir)
        if not os.path.exists(self.vid_out_dir):
            os.mkdir(self.vid_out_dir)

    '''
    Delete figure directory and its contents
    '''
    def clean_up(self):
        shutil.rmtree(self.fig_out_dir)

        
