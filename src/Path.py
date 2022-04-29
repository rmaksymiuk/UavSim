

class Path:
    def __init__(self, points=[], speeds=[], times=[]):
        '''
        The path class takes as argument a list of points,
        speeds, and times. These lists can be interpretted as directions
        to pursue a given point, p0, with a given speed, s0, and once that point is 
        reached, stay at that point for a given time. 
        '''
        if len(points) != len(speeds):
            raise ValueError("Must be same number of speeds as points in the path class")
        self.points = points
        self.speeds = speeds
        self.times = times

    def empty(self):
        return len(self.points) == 0
