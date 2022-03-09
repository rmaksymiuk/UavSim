import numpy as np
import pandas as pd

import util
from Vector import Vec2d, Vec3d
from Path import Path
from shapely.geometry import Polygon, Point, LineString
from shapely.ops import unary_union
from mpl_toolkits.mplot3d.art3d import Poly3DCollection

class UAV:
    '''
    Initialize the state of the UAV 
    '''
    def __init__(self, config):
        self.pos =  util.set_or_err(config, 'init_pos')
        self.init_energy = util.set_or_err(config, 'init_energy')
        self.name = util.set_or_err(config, 'name')

        self.set_path(util.set_or_default(config, 'init_path', None))
        self.velocity = util.set_or_default(config, 'init_vel', Vec3d())
        # Set the UAV direction based on its velocity
        if self.velocity.mag() == 0:
            self.dir = Vec3d(0, 1, 0).unit()
        else:
            self.dir = self.velocity.unit()

        self.fps = util.set_or_default(config, 'fps', 30)
        self.max_dist_drone = util.set_or_default(config, 'max_dist_drone', None)
        self.max_dist_base = util.set_or_default(config, 'max_dist_base', None)
        self.role = util.set_or_default(config, 'role', None)
        # Angles should be provided in radians
        self.lat_angle = util.set_or_default(config, 'lat_angle', 0.785) # associated with x #radians
        self.horiz_angle = util.set_or_default(config, 'horiz_angle', 0.785) # associated with #radians 
        self.max_rot_vel = util.set_or_default(config, 'max_rot_velocity', 0.174) # radians
        self.cs_area = util.set_or_default(config, 'cs_area', 0.1) # used for drag calculation
        self.speed_cost = util.set_or_default(config, 'speed_cost', default_speed_cost)
        self.shark_detected = util.set_or_default(config, 'shark_detected', default_shark_detected)
        # If a uav spots a shark within this buffer of places where it already thinks it saw a shark,
        # the spotting will not be reported to the Planner class
        self.already_seen_buffer = util.set_or_default(config, 'seen_buffer', 20)

        # Defining Thrust
        self.f_thrust = util.set_or_default(config, 'f_thrust', 20)
        self.b_thrust = util.set_or_default(config, 'b_thrust', 0)
        self.u_thrust = util.set_or_default(config, 'u_thrust', 20)
        self.d_thrust = util.set_or_default(config, 'd_thrust', 0)
        self.l_thrust = util.set_or_default(config, 'l_thrust', 2)
        self.r_thrust = util.set_or_default(config, 'r_thrust', 2)

        self.env_force= None # Set by environment
        self.color = None # Set by environment
        self.thrust = Vec3d()
        self.energy_used = 0.0
        self.time_since_frame = 0.0
        self.last_plot_time = 0.0
        self.area_covered = Polygon([]) # Using Shapely to manage area covered
        self.spotted = Polygon([]) # If a UAV spots a shark, it won't report it if it has already seen it

        self.history = [] # Contains a list of points the UAV has traveled to 
        self.TPs = [] # Contains [Polygon, time] tuples where a UAV spotted a shark
        self.FNs = [] # Contains [Polygon, time] typles where a UAV failed to spot a shark
        self.FPs = [] # Contains [Polygon, time] tuples where a UAV spotted a nonexistant shark


    '''
    Sets the path of the UAV to the given list of points. Also, changes the UAV's
    velocity to follow the path
    '''
    def set_path(self, path):
        if not path:
            self.path = None
            return
        if path.empty(): 
            self.path = None
            return
        self.path = path
        new_point = self.path.points[0]
        new_speed = self.path.speeds[0]
        self.update_vel(self.pos, new_point, new_speed)


    '''
    Returns a rectangle representing the frame captured by the UAV
    For now this function will assume that the drone is always upright. it will
    not account for the tilting of the drone to turn 
    '''
    def get_frame(self):
        x_dist = self.pos.z * np.tan(self.lat_angle)
        y_dist = self.pos.z * np.tan(self.horiz_angle)

        lat_dir = self.dir.to_Vec2d().unit()
        horiz_dir = lat_dir.rotate_90()

        lat_to_add = lat_dir.scale(x_dist)
        horiz_to_add = horiz_dir.scale(y_dist)

        pos2d = self.pos.to_Vec2d()
        
        return Polygon([
            (pos2d - lat_to_add - horiz_to_add).to_point(),
            (pos2d - lat_to_add + horiz_to_add).to_point(),
            (pos2d + lat_to_add + horiz_to_add).to_point(),
            (pos2d + lat_to_add - horiz_to_add).to_point(),
            (pos2d - lat_to_add - horiz_to_add).to_point()])

    '''
    Allows the Planner to ping the uav asking for its frame size at a given height
    '''
    def get_frame_size(self, height):
        x_dist = height * np.tan(self.lat_angle)
        y_dist = height * np.tan(self.horiz_angle)
        return x_dist * 2, y_dist * 2

    '''
    Steps the UAV through the environment 
        - timestep is the amount of time to advance the UAV state
        - total_time is the amount of time since the start of the simulation
        - sharks is a list of shark objects representing the sharks in the environment
    '''
    def step(self, timestep, total_time, sharks):
        self.naive_adjust_accel()
        self.take_vel_step(timestep)
        new_pos = self.pos + self.velocity.scale(timestep)

        # If the UAV is pursuing a path
        if self.path:

            step_taken = LineString([self.pos.to_point(), new_pos.to_point()]).buffer(1)

            # We can say that the UAV stops when it reaches its goal
            # UAV needs to change its velocity to pursue the next goal
            if step_taken.intersects(self.path.points[0].to_point()):
                print('%10s' % self.name, 'reached goal.', '%4d' % (len(self.path.points) - 1), 'to go.')
                new_pos = self.path.points.pop(0)
                self.path.speeds.pop(0)
                self.history.append(new_pos)


                # If the UAV still has tasks
                if not self.path.empty():
                    new_speed = self.path.speeds[0]
                    self.update_vel(new_pos, self.path.points[0], new_speed)

                # If the UAV has completed all tasks
                else:
                    self.update_vel(new_pos, new_pos, 0)
                    self.path = None

        self.pos = new_pos
        self.energy_used += (timestep * self.speed_cost(self.thrust))
        self.time_since_frame += timestep

        spotted_at = []
        # Check if the UAV should take a photo
        if (self.time_since_frame > (1 / self.fps)) and self.pos.z > 0:
            self.time_since_frame = 0
            frame = self.get_frame()
            self.area_covered = unary_union([self.area_covered, frame])
            for shark in sharks:
                detect_type, detected_pos = self.shark_detected(self, shark)
                if detect_type == 'TP':
                    shark.spotted(self, total_time)
                    self.TPs.append((frame, total_time))
                    detected_as_point = detected_pos.to_point()
                    if not self.spotted.contains(detected_as_point):
                        self.spotted = unary_union([self.spotted, detected_as_point.buffer(self.already_seen_buffer)])
                        spotted_at.append(detected_pos)
                elif detect_type =='FP':
                    detected_as_point = detected_pos.to_point()
                    self.FPs.append((frame, total_time))
                    spotted_at.append(detected_pos)
                    if not self.spotted.contains(detected_as_point):
                        self.spotted = unary_union([self.spotted, detected_as_point.buffer(self.already_seen_buffer)])
                        spotted_at.append(detected_pos)
                elif detect_type =='FN':
                    self.FNs.append((frame, total_time))
        return spotted_at

    '''
    Takes a step in velocity while checking if the simulation passed
    the velocity it was aiming for
    '''
    def take_vel_step(self, timestep):
        if not self.path:
            desired_vel = Vec3d()
        else:
            point_diff = self.path.points[0] - self.pos
            desired_vel = point_diff.unit().scale(self.path.speeds[0])

        cur_vel = self.velocity
        new_vel = self.velocity + (self.env_force + self.thrust).scale(timestep)
        if LineString([cur_vel.to_point(), new_vel.to_point()]).buffer(1).intersects(desired_vel.to_point()):
            self.velocity = desired_vel
        else:
            self.velocity = new_vel 
        if self.velocity.mag() > 0:
            self.dir = self.velocity.unit()

    '''
    Naively adjusts the accelleration by applying maximum thrust to align it 
    with the next position vector
    '''
    def naive_adjust_accel(self):
        if not self.path:
            desired_vel = Vec3d()
        else:
            point_diff = self.path.points[0] - self.pos
            desired_vel = point_diff.unit().scale(self.path.speeds[0])

        pursue_diff = desired_vel - self.velocity
        thrust_vecs = self.get_thrust_vectors()

        env_contrib = self.env_force.project_onto(pursue_diff)
        if env_contrib.vec @ pursue_diff.vec > 0:
            correct_env = self.env_force - env_contrib
        else:
            correct_env = self.env_force

        self.thrust = Vec3d()
        for tv in thrust_vecs:
            # First see if we should contribute thrust to balancing environmental forces
            if correct_env.mag() > 0:
                env_dir = tv.project_onto(correct_env)
                if env_dir.vec @ correct_env.vec > 1:
                    if env_dir.mag() > correct_env.mag():
                        remain_tv = tv - env_dir.unit().scale(correct_env.mag())
                        correct_env = Vec3d()
                    else:
                        correct_env = correct_env - env_dir
                        remain_tv = tv - env_dir
                else:
                    remain_tv = tv
            else:
                remain_tv = tv

            # The rest of the thrust vector should be contributed to the current thrust
            if pursue_diff.mag() > 0:
                for_thrust = remain_tv.project_onto(pursue_diff)
                if for_thrust.vec @ pursue_diff.vec > 0:
                    self.thrust = self.thrust + for_thrust

    '''
    Gets the possible thrust vectors for the uav
    '''
    def get_thrust_vectors(self):
        simple_dir = Vec3d(self.dir.x, self.dir.y, 0).unit()
        vecs = [
            Vec3d(0, 0, self.u_thrust),
            Vec3d(0, 0, self.d_thrust),
            simple_dir.scale(self.f_thrust),
            simple_dir.rotate_z(90).scale(self.l_thrust),
            simple_dir.rotate_z(180).scale(self.b_thrust),
            simple_dir.rotate_z(270).scale(self.r_thrust)
        ]
        return vecs


    '''
    Calculates the Drag of the UAV
        - Force vector in the opposite direction of velocity
        - Assumes no wind
    '''
    def calc_drag(self):
        v_mag = self.velocity.mag()
        if v_mag == 0:
            return Vec3d()
        d_force = 0.5 * 1.225 * (v_mag ** 2) * 1.2 * self.cs_area
        return self.velocity.unit().scale(-1 * d_force)

    '''
    Updates the velocity and direction of the current drone
    '''
    def update_vel(self, cur_pos, new_pos, speed):
        diff = new_pos - cur_pos
        if diff.is_zero():
            self.velocity = Vec3d()
        else:
            self.velocity = diff.unit().scale(speed)
            self.dir = self.velocity.unit()

    '''
    Plot the progress of this UAV on the given axis
    '''
    def plot_uav(self, ax, total_time):
        try:
            covered_pgons = list(self.area_covered)
        except TypeError:
            covered_pgons = [self.area_covered]
        for pgon in covered_pgons:
            if pgon.area > 0:
                ax.add_collection3d(Poly3DCollection(util.convert_3d(pgon), alpha=0.25, color=self.color))

        # Plot True Positives if they haven't been plotted
        for pgon, time in self.TPs:
            if time > self.last_plot_time:
                if pgon.area > 0:
                    ax.add_collection3d(Poly3DCollection(util.convert_3d(pgon), alpha=0.5, color='green'))

        # Plot False positives if they haven't been plotted
        for pgon, time in self.FPs:
            if time > self.last_plot_time:
                if pgon.area > 0:
                    ax.add_collection3d(Poly3DCollection(util.convert_3d(pgon), alpha=0.5, color='yellow'))

        # plot False Negatives if they haven't been plotted 
        for pgon, time in self.FNs:
            if time > self.last_plot_time:
                if pgon.area > 0:
                    ax.add_collection3d(Poly3DCollection(util.convert_3d(pgon), alpha=0.5, color='red'))

        # Plot location of UAV
        ax.scatter([self.pos.x], [self.pos.y], [self.pos.z], color = self.color, label=self.name)
        self.last_plot_time = total_time

    '''
    Sets the environmental force vector
    '''
    def set_env_force(self, env_force):
        self.env_force = env_force

    '''
    Returns a dataframe containing the stats for the current UAV
    '''
    def get_stats(self):
        TP = len(self.TPs)
        FP = len(self.FPs)
        FN = len(self.FNs)
        index = ['Name', 'TP', 'FP', 'FN', 'Energy_Used', 'Area_Covered']
        return pd.Series([self.name, TP, FP, FN,  self.energy_used, self.area_covered], index=index)


    '''
    Sets the color of the uav
    '''
    def set_color(self, color):
        self.color = color


'''
Now we can define speed cost in terms of thrust magnatude
Returns energy / time
'''
def default_speed_cost(thrust):
    return thrust.mag()


'''
Speed cost function to be used if none is supplied.
Return the energy use as a rate. Returns Energy/Time used
'''
def old_default_speed_cost(velocity):
    base_cost = 0.1
    up_cost = 1
    down_cost = 0
    lat_cost = 0.5

    energy_cost = 0
    if velocity.z > 0:
        energy_cost += (up_cost * velocity.z)
    else:
        energy_cost += (down_cost * np.abs(velocity.z))
    energy_cost += (lat_cost * np.abs(velocity.x))
    energy_cost += (lat_cost * np.abs(velocity.y))
    return energy_cost
    

'''
Object detection function to be used if none is supplied
Params:
    - uav: Uav object
    - sharks: List of shark objects from the environment

Returns the location of the shark if it was spotted in the frame
If there is a "false positive" return a random location in the frame

'''
def default_shark_detected(uav, shark):
    true_pos_optimal = 0.9 # Performance in an ideal environemnt
    false_pos_optimal = 0.001 # False postive rate in an ideal environment

    cur_frame = uav.get_frame()
    corner = list(cur_frame.exterior.coords)[0]
    max_angle = util.get_angle_to(uav.pos, Vec2d(corner[0], corner[1]))

    if cur_frame.contains(shark.pos.to_point()):
        ## Shark is in frame
        shark_angle = util.get_angle_to(uav.pos, shark.pos)
        ## Calculate the probability that we see the shark
        angle_weight = 0.2
        drone_velocity_weight = 0.2
        drone_height_weight = 0.2
        shark_depth_weight = 0.4

        angle_score = shark_angle / max_angle
        drone_velocity_score = util.get_sig(bal_point=30, scaling_val=-5)(uav.velocity.mag())
        drone_height_score = util.get_sig(bal_point=30, scaling_val=-5)(uav.pos.z)
        shark_depth_score = util.get_sig(bal_point= -5, scaling_val=5)(shark.pos.z)

        prob = 0
        prob += angle_weight * angle_score
        prob += drone_velocity_weight * drone_velocity_score
        prob += drone_height_weight * drone_height_score 
        prob += shark_depth_weight * shark_depth_score
        prob *= true_pos_optimal

        detected = np.random.choice([0, 1], size=1, p=[1 - prob, prob])[0]
        return ('TP', shark.pos.to_Vec2d()) if detected == 1 else ('FN', None)

    else:
        ## Shark is not in the frame
        drone_velocity_weight = 0.5
        drone_height_weight = 0.5
        drone_velocity_score = util.get_sig(bal_point=30, scaling_val=-5)(uav.velocity.mag())
        drone_height_score = util.get_sig(bal_point=30, scaling_val=-5)(uav.pos.z)

        prob = 0
        prob += drone_velocity_weight * drone_velocity_score
        prob += drone_height_weight * drone_height_score 
        
        prob = 1 - (1 - false_pos_optimal) * prob
        detected = np.random.choice([0, 1], size=1, p=[1 - prob, prob])[0]

        return ('TN', None) # For now we wont worry about false positives

        if detected == 1:
            return ('FP', util.gen_random(cur_frame, n_points=1)[0])
        else:
            return ('TN', None)
