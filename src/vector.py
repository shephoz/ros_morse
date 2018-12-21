import math

class Vector:
    # attributes
    # - x : float
    # - y : float
    # properties
    # - rad : float
    # methods
    # - path_from(from) : Vector
    # - path_to  (from) : Vector
    # - normalize() : Vector
    # - norm()       : float
    # - norm(vector) : float
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

    def __str__(self):
        return "({:.2f},{:.2f})".format(self.x, self.y)

    @property
    def rad(self):
        return math.atan2(self.y, self.x)

    def set(self,arg1,arg2=None):
        if(arg2 is None):
            if(isinstance(arg1,Vector)):
                # set(Vector)
                self.x = arg1.x
                self.y = arg1.y
        else:
            # set(x;float, y;float)
            self.x = arg1
            self.y = arg2

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

    def plus(self,vector):
        r = Vector()
        r.x = self.x + vector.x
        r.y = self.y + vector.y
        return r

    def normalize(self):
        return Vector(self.x/self.norm(), self.y/self.norm())

    def norm(self,vector=None):
        if(vector is None):
            return math.hypot(self.x, self.y)
        else:
            return math.hypot(self.x - vector.x, self.y - vector.y)

    def rotate(self,rad):
        r = Vector()
        r.x = math.cos(rad)*self.x - math.sin(rad)*self.y
        r.y = math.sin(rad)*self.x + math.cos(rad)*self.y
        return r

    def scalar_prod(self,scalar):
        r = Vector()
        r.x = self.x * scalar
        r.y = self.y * scalar
        return r

    def para(self,opposite_vector):
        rad = self.path_to(opposite_vector).rad * -1
        return math.cos(rad)*self.x - math.sin(rad)*self.y

    def perp(self,opposite_vector):
        rad = self.path_to(opposite_vector).rad * -1
        return math.sin(rad)*self.x + math.cos(rad)*self.y
