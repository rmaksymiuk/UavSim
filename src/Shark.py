
from Vector import Vec2d, Vec3d
import util


class Shark:
    '''
    Initialize the state of the shark
    '''
    def __init__(self, config):
        self.pos = util.set_or_err(config, 'init_pos')
        self.velocity = util.set_or_default(config, 'init_vel', Vec3d())
        self.spotted_by = [] # List of (UAV, time) tuples where the shark was spotted 

    '''
    Make the shark take a step in time
    '''
    def step(self, time):
        self.pos = self.pos + self.velocity.scale(time)

    '''
    Tell the shark it was spotted by a particular UAV at a particular time
    '''
    def spotted(self, uav, time):
        self.spotted_by.append((uav, time))

    '''
    Plot the shark. It is red if it hasn't been spotted, and green if it has
    '''
    def plot_shark(self, ax):
        if len(self.spotted_by) > 0:
            ax.scatter([self.pos.x], [self.pos.y], [self.pos.z], color='green', marker='^', label="Spotted Shark")
        else:
            ax.scatter([self.pos.x], [self.pos.y], [self.pos.z], color='red', marker='^', label="Not Spotted Shark")
