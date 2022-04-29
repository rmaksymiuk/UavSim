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
        self.focus = util.set_or_err(config, 'focus')

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
        self.mass = util.set_or_default(config, 'mass', 2)
        self.role = util.set_or_default(config, 'role', None)
        # Angles should be provided in radians
        self.lat_angle = util.set_or_default(config, 'lat_angle', 0.785) # associated with x #radians
        self.horiz_angle = util.set_or_default(config, 'horiz_angle', 0.785) # associated with #radians 
        self.cs_area = util.set_or_default(config, 'cs_area', 0.1) # used for drag calculation
        self.speed_cost = util.set_or_default(config, 'speed_cost', default_speed_cost)
        self.object_detected = util.set_or_default(config, 'object_detected', default_object_detected)
        # If a uav spots an object within this buffer of places where it already thinks it saw a object,
        # the spotting will not be reported to the Planner class
        self.already_seen_buffer = util.set_or_default(config, 'seen_buffer', 20)
        self.energy_limit = util.set_or_default(config, 'energy_limit', None) # Energy Limit for UAVs

        # Defining Thrust
        self.f_thrust = util.set_or_default(config, 'f_thrust', 40)
        self.b_thrust = util.set_or_default(config, 'b_thrust', 10)
        self.u_thrust = util.set_or_default(config, 'u_thrust', 40)
        self.d_thrust = util.set_or_default(config, 'd_thrust', 0)
        self.l_thrust = util.set_or_default(config, 'l_thrust', 40)
        self.r_thrust = util.set_or_default(config, 'r_thrust', 40)

        self.env_force= None # Set by environment
        self.color = None # Set by environment
        self.thrust = Vec3d()
        self.energy_used = 0.0
        self.time_since_frame = 0.0
        self.last_plot_time = 0.0
        self.waiting = False # Allow the UAV to stop at a given point
        self.wait_time = 0.0
        self.area_covered = Polygon([]) # Using Shapely to manage area covered
        self.spotted = Polygon([]) # If a UAV spots an object, it won't report it if it has already seen it

        self.history = [] # Contains a list of points the UAV has traveled to 
        self.TPs = [] # Contains [Polygon, time] tuples where a UAV spotted a object
        self.FNs = [] # Contains [Polygon, time] typles where a UAV failed to spot a object
        self.FPs = [] # Contains [Polygon, time] tuples where a UAV spotted an object that was not an object of interest


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
        self.speed_func = self.get_speed_func()


    '''
    Return a scaling function for desired speed
    start_point and end_point are Vec3d. Uses the current path
    of the UAV to get a speed function for the leg
    '''
    def get_speed_func(self):
        path_len = (self.path.points[0] - self.pos).mag()
        if len(self.path.points) > 1:
            next_leg = (self.path.points[1] - self.path.points[0]).unit().vec
        else:
            next_leg = Vec3d().vec
        dir_weight = next_leg @ (self.path.points[0] - self.pos).unit().vec
        sig_fun = util.get_sig(0.99 * path_len, 0.05)
        def speed_func(dist):
            next_weight = sig_fun(path_len - dist)
            return (1 - next_weight) * self.path.speeds[0] + (next_weight) * dir_weight * self.path.speeds[0]
        return lambda x: self.path.speeds[0]
        return speed_func

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
        - env_objects is a list of objects representing the objects in the environment
        - control_pos is the position of the control station
    '''
    def step(self, timestep, total_time, env_objects, control_pos, winds):
        tot_force = self.env_force + self.calc_drag(winds)
        self.naive_adjust_accel(tot_force, timestep)
        self.take_vel_step(tot_force, timestep)
        new_pos = self.pos + self.velocity.scale(timestep)

        # If the UAV is pursuing a path
        if self.path:

            step_taken = LineString([self.pos.to_point(), new_pos.to_point()]).buffer(1)

            # We can say that the UAV stops when it reaches its goal
            # UAV needs to change its velocity to pursue the next goal
            if step_taken.intersects(self.path.points[0].to_point()):
                if self.path.times[0] > 0:
                    # Drone has to wait
                    if not self.waiting:
                        self.waiting = True
                        self.path.speeds[0] /= 40 
                    else:
                        self.path.times[0] -= timestep
                else:
                    print('%10s' % self.name, 'reached goal.', '%4d' % (len(self.path.points) - 1), 'to go.')
                    self.waiting = False
                    new_pos = self.path.points.pop(0)
                    self.path.speeds.pop(0)
                    self.path.times.pop(0)
                self.history.append(new_pos)
                if self.path.empty():
                    self.path = None
                else:
                    self.speed_func = self.get_speed_func()

        if not self.pos.to_point().buffer(1).intersects(control_pos.to_point()):
            self.energy_used += (timestep * self.speed_cost(self.thrust))
            if self.energy_limit and self.energy_used > self.energy_limit:
                raise ValueError('UAV: ' + self.name + ' Expended All Energy')
        self.pos = new_pos
        self.time_since_frame += timestep

        spotted_at = []
        # Check if the UAV should take a photo
        if (self.time_since_frame > (1 / self.fps)) and self.pos.z > 0:
            self.time_since_frame = 0
            frame = self.get_frame()
            self.area_covered = unary_union([self.area_covered, frame])
            for env_object in env_objects:
                if not env_object.visible:
                    continue
                detect_type, detected_pos = self.object_detected(self, env_object)
                if detect_type == 'TP':
                    env_object.spotted(self, total_time) # Only say the object was spotted if it is the obj of focus
                    self.TPs.append((frame, total_time))
                    detected_as_point = detected_pos.to_point()
                    if not self.spotted.contains(detected_as_point):
                        self.spotted = unary_union([self.spotted, detected_as_point.buffer(self.already_seen_buffer)])
                        spotted_at.append(detected_pos)
                elif detect_type =='FP':
                    detected_as_point = detected_pos.to_point()
                    self.FPs.append((frame, total_time))
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
    def take_vel_step(self, tot_force, timestep):
        desired_vel = Vec3d()
        if self.path:
            point_diff = self.path.points[0] - self.pos
            p_mag = point_diff.mag()
            if p_mag > 0:
                desired_vel = point_diff.unit().scale(min(p_mag / timestep, self.speed_func(p_mag)))
        cur_vel = self.velocity
        new_vel = self.velocity + (tot_force + self.thrust).scale(timestep / self.mass)
        if LineString([cur_vel.to_point(), new_vel.to_point()]).buffer(1).intersects(desired_vel.to_point()):
            self.velocity = desired_vel
        else:
            self.velocity = new_vel 
        if self.velocity.mag():
            self.dir = self.velocity.unit()

    '''
    Naively adjusts the accelleration by applying maximum thrust to align it 
    with the next position vector. We know that the maximum acceleration that
    we want is the acceleration that will achieve the desired velocity in the 
    next timestep.
    '''
    def naive_adjust_accel(self, tot_force, timestep):
        desired_vel = Vec3d()
        if self.path: 
            point_diff = self.path.points[0] - self.pos
            p_mag = point_diff.mag()
            if p_mag > 0:
                desired_vel = point_diff.unit().scale(min(p_mag / timestep, self.speed_func(p_mag)))

        pursue_diff = (desired_vel - self.velocity).scale(self.mass / timestep)
        thrust_vecs = self.get_thrust_vectors()

        env_contrib = tot_force.project_onto(pursue_diff)
        if env_contrib.vec @ pursue_diff.vec > 0:
            if env_contrib.mag() > pursue_diff.mag():
                env_contrib -= pursue_diff
                pursue_diff = Vec3d()
            else:
                pursue_diff -= env_contrib
            correct_env = tot_force - env_contrib
        else:
            correct_env = tot_force 

        self.thrust = Vec3d()
        for tv in thrust_vecs:
            # First see if we should contribute thrust to balancing environmental forces
            if correct_env.mag() > 0:
                env_dir = tv.project_onto(correct_env)
                if env_dir.vec @ correct_env.vec < 0:
                    if env_dir.mag() > correct_env.mag():
                        part_contrib = env_dir.unit().scale(correct_env.mag())
                        self.thrust += part_contrib
                        remain_tv = tv - part_contrib 
                        correct_env = Vec3d()
                    else:
                        correct_env = correct_env + env_dir
                        self.thrust += env_dir
                        remain_tv = tv - env_dir
                else:
                    remain_tv = tv
            else:
                remain_tv = tv

            # The rest of the thrust vector should be contributed to the current thrust
            if pursue_diff.mag() > 0:
                for_thrust = remain_tv.project_onto(pursue_diff)
                if for_thrust.vec @ pursue_diff.vec > 0:
                    if for_thrust.mag() > pursue_diff.mag():
                        pursue_diff = Vec3d()
                        self.thrust += for_thrust.unit().scale(pursue_diff.mag())
                    else:
                        pursue_diff -= for_thrust
                        self.thrust += for_thrust

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
    def calc_drag(self, winds):
        a_vel = Vec3d()
        for w in winds:
            if w.boundary.covers(self.pos.to_point()):
                a_vel += w.velocity
        v_diff = a_vel - self.velocity
        if v_diff.mag() == 0:
            return Vec3d()
        v_diff_sqr = v_diff.vec @ v_diff.vec
        d_force = 0.5 * 1.225 * (v_diff.unit().scale(v_diff_sqr).vec) * 1.2 * self.cs_area
        return Vec3d(d_force[0], d_force[1], d_force[2]) 

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
        index = ['Name', 'Focus', 'TP', 'FP', 'FN', 'Energy_Used', 'Area_Covered']
        return pd.Series([self.name, self.focus, TP, FP, FN,  self.energy_used, self.area_covered], index=index)


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
Object detection function to be used if none is supplied
Params:
    - uav: Uav object
    - env_object: Object from the environment  

Returns the location of the object if it was spotted in the frame
'''
def default_object_detected(uav, env_object):
    true_pos_optimal = 0.95 # Performance in an ideal environemnt

    cur_frame = uav.get_frame()
    corner = list(cur_frame.exterior.coords)[0]
    max_angle = util.get_angle_to(uav.pos, Vec2d(corner[0], corner[1]))

    is_focus = env_object.type == uav.focus
    performance_multiplyer = env_object.get_multiplier(is_focus)
    actual_performance = true_pos_optimal * performance_multiplyer

    if cur_frame.contains(env_object.pos.to_point()):
        ## Object is in frame
        object_angle = util.get_angle_to(uav.pos, env_object.pos)
        ## Calculate the probability that we see the object
        angle_weight = 0.1
        drone_velocity_weight = 0.2
        drone_height_weight = 0.5
        object_depth_weight = 0.1

        angle_score = object_angle / max_angle
        drone_velocity_score = util.get_sig(bal_point=30, scaling_val=-5)(uav.velocity.mag())
        drone_height_score = util.get_sig(bal_point=30, scaling_val=-5)(uav.pos.z)
        object_depth_score = util.get_sig(bal_point= -5, scaling_val=5)(env_object.pos.z)

        prob = 0
        prob += angle_weight * angle_score
        prob += drone_velocity_weight * drone_velocity_score
        prob += drone_height_weight * drone_height_score 
        prob += object_depth_weight * object_depth_score

        # A good prob score should lead to a decreased likelyhood of false positives
        if is_focus:
            prob *= actual_performance
        else:
            prob = (1 - prob) * actual_performance


        detected = np.random.choice([0, 1], size=1, p=[1 - prob, prob])[0]
        if detected == 1:
            if is_focus:
                return ('TP', env_object.pos.to_Vec2d())
            return ('FP', env_object.pos.to_Vec2d())
        if is_focus:
            return ('FN', None)
    return ('TN', None) 

