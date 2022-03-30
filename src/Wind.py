## Object representation of wind in the simulation
## The main purpose of this module is for plotting wind

import numpy as np
from Vector import Vec2d, Vec3d
import util 
from shapely.geometry import LineString, Point

class Wind:
    def __init__(self, boundary, velocity, env_bound, first=False):
        self.boundary = boundary
        self.velocity = velocity #Vec3d
        self.planer_vel = Vec3d(self.velocity.x, self.velocity.y, 0)
        self.indicators, self.indicator_base, self.indicator_height, self.goal_point \
                = self.get_indicators(env_bound)
        self.add_height = self.planer_vel.unit().scale(self.indicator_height)
        self.first = first

    '''
    Steps the wind through the environment. This is just for the indicators
    '''
    def step(self, timestep):
        new_indicators = []
        for i, ind in enumerate(self.indicators):
            new_point = ind + self.planer_vel.scale(timestep)
            if (new_point - self.goal_point).vec @ self.planer_vel.vec >= 0:
                new_indicators.append(self.indicator_base[i])
            else:
                new_indicators.append(new_point)
        self.indicators = new_indicators

    '''
    Plots wind in the environment. 
    '''
    def plot(self, ax):
        first_ind = True
        for ind in self.indicators:
            end = ind + self.add_height
            ls = LineString([ind.to_point(), end.to_point()])
            if not self.boundary.covers(ls):
                continue
            if self.first and first_ind:
                ax.plot([ind.x, end.x], [ind.y, end.y], [ind.z, end.z], alpha=0.5, color='black', label='wind')
                first_ind = False
            else:
                ax.plot([ind.x, end.x], [ind.y, end.y], [ind.z, end.z], alpha=0.5, color='black')



    '''
    Gets the indicators for the wind object for plotting.
    '''
    def get_indicators(self, env_bound):
        planer_dir = self.velocity.to_Vec2d()
        rot_planer_dir = planer_dir.rotate_90()
        ind_width = 0.05 * np.sqrt(env_bound.area)
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
            hor_dots.append(hor_proj.vec @ rot_planer_dir.vec) 
            hor_projections.append(hor_proj)

        # Find Base Start Points
        start_ind = hor_projections[np.argmin(hor_dots)]
        end_ind = hor_projections[np.argmax(hor_dots)]
        ver_start = ver_projections[np.argmin(ver_dots)]
        ver_end = ver_projections[np.argmax(ver_dots)]

        # Calculate start lines
        start_ival = 0.1 * np.sqrt(env_bound.area)
        ver_line = LineString([ver_start.to_point(), ver_end.to_point()])
        line_points = [ver_line.interpolate(p) for p in np.arange(start=0, stop=ver_line.length, step=start_ival)]
        v_points = [Vec2d(l.x, l.y) for l in line_points]
        start_lines = []
        for v in v_points:
            sp1 = util.intersection(start_ind, planer_dir, v, rot_planer_dir)
            sp2 = util.intersection(end_ind, planer_dir, v, rot_planer_dir)
            start_lines.append(LineString([sp1.to_point(), sp2.to_point()]))

        # Get initial points
        start_locs = np.arange(start=0, stop=start_lines[0].length + ind_width, step=ind_width)
        indicator_starts = []
        indicator_base = []
        for i, loc in enumerate(start_locs):
            base_point = start_lines[0].interpolate(loc)
            base_vec = Vec3d(base_point.x, base_point.y, 0)
            for j, line in enumerate(start_lines):
                if (i % 2 == 0 and j % 2 == 0) or (i % 2 != 0 and j % 2 != 0):
                    cur_point = line.interpolate(loc) 
                    indicator_starts.append(Vec3d(cur_point.x, cur_point.y, 0))
                    indicator_base.append(base_vec)
        return indicator_starts, indicator_base, ind_height, Vec3d(ver_end.x, ver_end.y, 0)
        


