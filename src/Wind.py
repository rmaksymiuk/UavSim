## Object representation of wind in the simulation
## The main purpose of this module is for plotting wind

import numpy as np
from Vector import Vec2d, Vec3d
import util
from shapely.geometry import LineString, Point

class Wind:
    def __init__(self, boundary, velocity, env_bound):
        self.boundary = boundary
        self.velocity = velocity #Vec3d
        self.planer_vel = Vec3d(self.velocity.x, self.velocity.y, 0)
        self.indicators, self.indicator_base, self.indicator_height = self.get_indicators(env_bound)
        self.add_height = self.planer_vel.unit().scale(self.indicator_height)


    '''
    Steps the wind through the environment. This is just for the indicators
    '''
    def step(self, timestep):
        new_indicators = []
        for i, ind in enumerate(self.indicators):
            new_point = ind + self.planer_vel.scale(timestep)
            cur_line = LineString(new_point.to_point(), (new_point + self.add_height).to_point())
            if not self.boundary.covers(cur_line):
                new_indicators.append(self.indicator_base[i])
            else:
                new_indicators.append(new_point)

    '''
    Plots wind in the environment. 
    '''
    def plot(self, ax):
        for ind in indicators:
            end = ind + self.add_height
            ax.plot([ind.x, end.x], [ind.y, end.y], [ind.z, end.z], alpha=0.5, color='black')


    '''
    Gets the indicators for the wind object for plotting.
    '''
    def get_indicators(self, env_bound):
        planer_dir = self.velocity.to_Vec2d()
        rot_planer_dir = planer_dir.rotate_90()
        ind_width = 0.1 * np.sqrt(env_bound.area)
        ind_height = 0.1 * np.sqrt(env_bound.area)
        ver_projections = []
        hor_projections = []
        ver_dots = []
        hor_dots = []
        for x, y in self.boundary.exterior.coords:
            ver_vec = Vec2d(x, y)
            ver_proj = ver_vec.project_onto(planer_dir)
            ver_dots.append(ver_proj.vec @ planer_dir.vec)
            ver_projections.append(ver_proj)
            hor_proj = ver_vec.project_onto(rot_planer_dir)
            hor_dots.append(hor_proj.vec @ rot_planer_dir) 
            hor_projections.append(hor_proj)

        # Find Base Start Points
        start_ind = hor_projections[np.argmin(hor_dots)]
        end_ind = hor_projections[np.argmax(hor_dots)]
        ver_start = ver_projections[np.argmin(ver_dots)]
        start_point_1 = util.intersection(start_ind, planer_dir, ver_start, rot_planer_dir)
        start_point_2 = util.intersection(end_ind, planer_dir, ver_start, rot_planer_dir)
        start_line = LineString([start_point_1.to_point(), start_point_2.to_point()])

        # Find Mid Start Points
        ver_end = ver_projections[np.argmax(ver_dots)]
        ver_mid = (ver_start + ver_end).scale(0.5)
        mid_point_1 = util.intersection(start_ind, planer_dir, ver_mid, rot_planer_dir)
        mid_point_2 = util.intersection(end_ind, planer_dir, ver_mid, rot_planer_dir)
        mid_line = LineString([mid_point_1.to_point(), mid_point_2.to_point()])

        # Get initial points
        start_locs = np.arange(0, start_line.length + ind_width)
        indicator_starts = []
        indicator_base = []
        for i, loc in enumerate(start_locs):
            cur_point = start_line.interpolate(loc)
            base_point = Vec3d(cur_point.x, cur_point.y, 0)
            indicator_base.append(base_point)
            if i % 2 == 0:
                indicator_starts.append(base_point)
            else:
                cur_point = mid_line.interpolate(loc)
                indicator_starts.append(Vec3d(cur_point.x, cur_point.y, 0))
        return indicator_starts, indicator_base, ind_height
        


