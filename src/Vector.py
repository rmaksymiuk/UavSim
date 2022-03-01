from shapely.geometry import Point
import numpy as np

class Vec2d:
    '''
    If vector components are not specified, they default to 0
    '''
    def __init__(self, x=0.0, y=0.0):
        self.vec = np.array([x, y], dtype='float32')
        self.x = x
        self.y = y

    ## Setters
    def set_x(self, x):
        self.x = x
        self.vec[0] = x
    def set_y(self, y):
        self.y = y
        self.vec[1] = y

    '''
    Initialize a 2d vector from a np array of len 3
    '''
    def from_vec(self, arr):
        if len(arr) != 2:
            raise ValueError("Np Arr input must be of len 2")
        return Vec2d(arr[0], arr[1])

    '''
    Allow ourselves to add vectors
    '''
    def __add__(self, other):
        result = self.vec + other.vec
        return Vec2d(result[0], result[1])

    '''
    Allow ourselves to subtract vectors
    '''
    def __sub__(self, other):
        result = self.vec - other.vec
        return self.from_vec(result)

    '''
    Scale each vector component by a scaler value
    '''
    def scale(self, scaler):
        result = self.vec * scaler
        return Vec2d(result[0], result[1])

    '''
    Convert the vector into a shapely point
    '''
    def to_point(self):
        return Point(self.x, self.y)

    '''
    Returns the magnitude of the vector
    '''
    def mag(self):
        return np.sqrt(self.vec @ self.vec)

    '''
    Returns true if all components of the vector are 0
    '''
    def is_zero(self):
        return self.x == 0 and self.y == 0

    '''
    Returns the unit vector for the particular vector
    '''
    def unit(self):
        if self.is_zero():
            return None
        return self.from_vec(self.vec / self.mag())

    '''
    Returns a vector rotated clockwise
    '''
    def rotate_90(self):
        return Vec2d().from_vec(self.vec @ np.array([[0, -1], [1, 0]]))

    '''
    Generates a random vector
    '''
    def gen_random(self):
        return Vec2d(np.random.random(), np.random.random())

    '''
    Convert the 2D vector into a 3d vector
    '''
    def to_Vec3d(self, z=0):
        return Vec3d(self.x, self.y, z)


class Vec3d:
    '''
    If vector components are not specified, they default to 0
    '''
    def __init__(self, x=0.0, y=0.0, z=0.0):
        self.vec = np.array([x, y, z], dtype='float32')
        self.x = x
        self.y = y
        self.z = z

    ## Setters
    def set_x(self, x):
        self.x = x
        self.vec[0] = x
    def set_y(self, y):
        self.y = y
        self.vec[1] = y
    def set_z(self, z):
        self.z = z
        self.vec[2] = z

    '''
    Initialize a 3d vector from a np array of len 3
    '''
    def from_vec(self, arr):
        if len(arr) != 3:
            raise ValueError("Np Arr input must be of len 3")
        return Vec3d(arr[0], arr[1], arr[2])

    '''
    Allow ourselves to add vectors
    '''
    def __add__(self, other):
        result = self.vec + other.vec
        return Vec3d(result[0], result[1], result[2])

    '''
    Allow ourselves to subtract vectors
    '''
    def __sub__(self, other):
        result = self.vec - other.vec
        return Vec3d().from_vec(result)

    '''
    Equality between 3d and 2d vectors
    '''
    def eq_2d(self, other):
        return self.x == other.x and self.y == other.y

    '''
    Scale each vector component by a scaler value
    '''
    def scale(self, scaler):
        result = self.vec * scaler
        return Vec3d(result[0], result[1], result[2])

    '''
    Convert the 3d vector into a 2d point object. Only use the
    first two components of the vector
    '''
    def to_point2d(self):
        return Point(self.x, self.y)

    '''
    Convert the 3d vector into a shapely Point object
    '''
    def to_point(self):
        return Point(self.x, self.y, self.z)
    

    '''
    Return the magnitude of the vector
    '''
    def mag(self):
        return np.sqrt(self.vec @ self.vec)

    '''
    Returns true if all componenets of the vector equal 0
    '''
    def is_zero(self):
        return self.x == 0 and self.y == 0 and self.z == 0

    '''
    Returns the unit vector for the particular vector
    '''
    def unit(self):
        if self.is_zero():
            print("I'm zero", self.vec)
            return None
        return self.from_vec(self.vec / self.mag())

    '''
    Returns a vec2d object with just the x and y componenets of the vec3d object
    '''
    def to_Vec2d(self):
        return Vec2d(self.x, self.y)

    '''
    Generates a random vector
    '''
    def gen_random(self):
        return Vec3d(np.random.random(), np.random.random(), np.random.random())

