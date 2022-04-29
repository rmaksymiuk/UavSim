
from Vector import Vec2d, Vec3d
import pandas as pd
import util


class Env_Object:
    '''
    Initialize the state of the object
    '''
    def __init__(self, config):
        self.name = util.set_or_err(config, 'name')
        self.pos = util.set_or_err(config, 'init_pos')
        self.type = util.set_or_err(config, 'type')
        self.velocity = util.set_or_default(config, 'init_vel', Vec3d())
        self.focus_performance = util.set_or_default(config, 'focus_performance', 1)
        self.non_focus_performance = util.set_or_default(config, 'non_focus_performance', 0.4)
        self.velocity = util.set_or_default(config, 'init_vel', Vec3d())
        self.visible_starts, self.visible_ends = util.set_or_default(config, 'visible', (None, None))
        self.spotted_by = [] # List of (UAV, time) tuples where the object was spotted 

        self.marker = None # assigned by environment
        self.first = False # assigned by environment

        self.visible=True
        self.total_time = 0.0

    '''
    Make the object take a step in time
    '''
    def step(self, time):
        self.pos = self.pos + self.velocity.scale(time)
        self.total_time += time
        if self.visible_starts is not None:
            while len(self.visible_starts) > 0 and self.total_time > self.visible_ends[0]:
                self.visible_starts = self.visible_starts[1:]
                self.visible_ends = self.visible_ends[1:]
            if len(self.visible_starts) == 0 or self.total_time < self.visible_starts[0]:
                self.visible = False
            else:
                self.visible = True


    '''
    Tell the object it was spotted by a particular UAV at a particular time
    '''
    def spotted(self, uav, time):
        self.spotted_by.append((uav, time))

    '''
    Plot the object. It is red if it hasn't been spotted, and green if it has
    '''
    def plot_object(self, ax):
        if not self.visible:
            return
        color = 'red'
        if len(self.spotted_by) > 0:
            color = 'green'
        label=None
        if self.first:
            label = self.type

        ax.scatter([self.pos.x], [self.pos.y], [self.pos.z], color=color, marker=self.marker, label=label)

    '''
    Return the UAVs that spotted the object
    '''
    def get_stats(self):
        # Get unique uavs
        new_spotted_by = list(set([uav.name for uav, time in self.spotted_by]))
        uav_tups = [(self.name, uav_name) for uav_name in new_spotted_by]
        return pd.DataFrame(uav_tups, columns=['Object', 'UAV'])

    '''
    Get the multiplier for this object. is_focus indicates whether or not
    this is the object of interest to the UAV conducting the multiplier query
    '''
    def get_multiplier(self, is_focus):
        if is_focus:
            return self.focus_performance
        return self.non_focus_performance

    '''
    Assignment conducted by the environment for plotting purposes. Makes sure 
    each obejct type gets its own marker
    '''
    def assign_marker(self, marker, first):
        self.first = first
        self.marker = marker
        





