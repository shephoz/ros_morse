import math

class Vector:
    # attributes
    # - x : float
    # - y : float
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

    def path_from(self,from):
        r = Vector()
        r.x = self.x - from.x
        r.y = self.y - from.y
        return

    def path_to(self,to):
        r = Vector()
        r.x = to.x - self.x
        r.y = to.y - self.y
        return

    def normalize(self):
        return Vector(self.x/self.norm(), self.y/self.norm())

    def norm(vector=None):
        if(vector is None):
            return math.hypot(self.x, self.y)
        else:
            return math.hypot(self.x - vector.x, self.y - vector.y)
