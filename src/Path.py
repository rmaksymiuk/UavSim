

class Path:
    def __init__(self, points=[], speeds=[]):
        if len(points) != len(speeds):
            raise ValueError("Must be same number of speeds as points in the path class")
        self.points = points
        self.speeds = speeds

    def empty(self):
        return len(self.points) == 0


