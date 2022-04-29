
from shapely.geometry import Point, Polygon
from shapely.ops import unary_union
from Vector import Vec2d, Vec3d
from Env_Object import Env_Object
import numpy as np
import glob, os, cv2

'''
If the key is in the config dictionary, return
the value from the config dictionary. If the key
is not in the config dictionary, return the default
value.
'''
def set_or_default(config, key, default):
    if key in config:
        return config[key]
    return default

'''
If the key is not in the config dictionary, raise an error
because the key must be in the config dictionary
'''
def set_or_err(config, key):
    try:
        return config[key]
    except KeyError as k:
        raise ValueError(key + ' must be in config dictionary')

'''
pos1 is a vec3D object 
pos2 is a vec2D object (restricted to movement on the x-y plane)
'''
def get_angle_to(pos1, pos2):
    x_dist = pos1.x - pos2.x
    y_dist = pos1.y - pos2.y
    base = np.sqrt(x_dist ** 2 + y_dist ** 2)
    return np.arctan(base / pos1.z)



'''
Given the problem constraints, returns a sigmoid function to 
model the problem and give a plausible score

Scaling Val describes the sensitivity of the score,
and bal point is the point in which the most mediocre score is given.
A negative scaling val indicates that smaller values should recieve a 
higher score
'''
def get_sig(bal_point, scaling_val, l_bound=0, u_bound=1):
    p_range = u_bound - l_bound
    def sig(x):
        num = np.exp(scaling_val * (x - bal_point))
        denom = np.exp(scaling_val * (x - bal_point)) + 1
        return p_range * num / (denom + 1) + l_bound
    return sig


'''
Randomly generate points within the given polygon
'''
def gen_random(poly, n_points):
    points = []
    minx, miny, maxx, maxy = poly.bounds
    while len(points) < n_points:
        x_loc = np.random.uniform(minx, maxx, size=1)[0]
        y_loc = np.random.uniform(miny, maxy, size=1)[0]
        pnt = Point(x_loc, y_loc)
        if poly.contains(pnt):
            points.append(Vec2d(x_loc, y_loc))
    return points


'''
Convert a polygon object to a list of tuples that can be plotted
on a 3d plot
'''
def convert_3d(poly, z=0):
    new_pts = []
    for x, y in list(poly.exterior.coords):
        new_pts.append((x, y, z))
    return [new_pts]


'''
Randomly spawn objects in a given environment going a certain speed
Assumes that velocities can only have a direction in the x, y plane. No z
Visible freq defines how often the object is visible and visible count defines
how many times the object is visible. Used with time_estimate, these can generate
random visibility intervals
'''
def spawn_objects(n, otype, poly, focus_perf=1, non_focus_perf=0.4, speed=1, 
        visible_freq=None, visible_count=None, time_estimate=None):


    start_points = gen_random(poly, n)
    env_objects = []
    visible_starts, visible_ends = (None, None)
    for i in range(n):
        if visible_freq and visible_count and time_estimate:
            visible_starts, visible_ends = get_visible_intervals(visible_freq, visible_count, time_estimate)
        rand_vel = Vec3d().gen_random()
        rand_vel.set_z(0)
        env_objects.append(Env_Object({
            'name': 'Object_' + str(i), 
            'type': otype,
            'init_pos': start_points[i].to_Vec3d(), 
            'focus_performance': focus_perf,
            'non_focus_performance': non_focus_perf,
            'init_vel': rand_vel.scale(speed),
            'visible': (visible_starts, visible_ends)}))
            
    return env_objects


'''
Over a given interval of time, return `count` intervals that cover `freq` proportion
of time.

The algorithm works like this. We create a list of chuncks where each chunk is a random
size but they all add up to the total time. While there are still chuncks, we insert
the chunks at a random location between the end of the last chunck, and the start of
the rest of the chuncks if they were squished at the end. 
'''
def get_visible_intervals(freq, count, time):
    chunks_bounds = np.sort(np.random.uniform(low=0, high=time * freq, size=count))
    chunks = chunks_bounds[1:] - chunks_bounds[:-1]
    last_end = 0
    starts = []
    ends = []
    for i, chunk in enumerate(chunks):
        insert_at = np.random.uniform(low=last_end, high=time - chunks[i:].sum())
        last_end = insert_at + chunk
        starts.append(insert_at)
        ends.append(last_end)
    return starts, ends



'''
Create a video from the generated plots
Assumes that lexographically sorting filenames in the
    directory is the correct seq
    ency for the video
'''
def create_video(video_name, image_dir, extention='png'):
    if not video_name.endswith('.mp4'):
        raise ValueError('video must end with mp4')
    images = sorted(glob.glob(os.path.join(image_dir, '*.' + extention)))
    frame=cv2.imread(images[0])
    height, width, layers = frame.shape

    # Define the codec and create VideoWriter object
    fourcc = cv2.VideoWriter_fourcc(*'mp4v') # Be sure to use lower case

    video = cv2.VideoWriter(video_name, fourcc, 20.0, (width, height))
    for image in images:
        video.write(cv2.imread(image))

    video.release()
    cv2.destroyAllWindows()


'''
Given a list of polygons, reuturn the unary union of all of the polygons
'''
def list_union(pgons):
    accum = Polygon([])
    for pgon in pgons:
        accum = unary_union([accum, pgon])
    return accum

'''
Given two points, and two velocities. Find the intersection of the lines.
Assume the points are two dimensional
'''
def intersection(p1, v1, p2, v2):
    trans_mat = np.array([[0, 1], [-1, 0]])
    l1 = v1.vec @ trans_mat
    l2 = v2.vec @ trans_mat
    l_mat = np.concatenate((l1[None, :], l2[None, :]))
    r1 = l1 @ p1.vec
    r2 = l2 @ p2.vec
    r_mat = np.array([r1, r2])
    result = np.linalg.solve(l_mat, r_mat)
    return Vec2d(result[0], result[1])






