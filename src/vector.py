import math

class Vector:
    # attributes
    # - x : float
    # - y : float
    # methods
    # - path_from(from) : Vector
    # - path_to  (from) : Vector
    # - normalize() : Vector
    # - norm()       : float
    # - norm(vector) : float
    # - tan() : float
    def __init__(self,arg1=None,arg2=None):
        # initialize x, y
        if(arg1 is None):
            # Vector()
            self.x = 0.0
            self.y = 0.0
        else:
            if(arg2 is None):
                if(isinstance(arg1,Point)):
                    # Vector(Point)
                    self.x = arg1.x
                    self.y = arg1.y
            else:
                # Vector(float,float)
                self.x = arg1
                self.y = arg2

    def set(self,x,y):
        self.x = x
        self.y = y

    def path_from(self,vector):
        r = Vector()
        r.x = self.x - vector.x
        r.y = self.y - vector.y
        return r

    def path_to(self,vector):
        r = Vector()
        r.x = vector.x - self.x
        r.y = vector.y - self.y
        return r

    def normalize(self):
        return Vector(self.x/self.norm(), self.y/self.norm())

    def norm(vector=None):
        if(vector is None):
            return math.hypot(self.x, self.y)
        else:
            return math.hypot(self.x - vector.x, self.y - vector.y)

    def tan(self):
        return math.atan2(self.y, self.x)
