from vector import Vector
from pose   import Pose

class Subject:
    # attributes
    # - pose  : Pose
    # - cache : Pose
    # - vel   : Vector
    # methods
    # - update_pose(pose : Pose) -> pose.set()
    # - calc_vel()
    # aliases
    # - gap_to  -> Pose
    # - path_to -> Vector
    # - perp -> Vector
    # - para -> Vector

    def __init__(self):
        self.pose  = Pose()
        self.cache = None
        self.vel   = Vector()

    def __str__(self):
        return "{} - {} - {}".format(self.pose, self.cache, self.vel)

    def update_pose(self,pose):
        self.pose.set(pose)

    def calc_vel(self):
        if(self.cache is None):
            self.cache = Pose()
            self.cache.set(self.pose)
        if(True or self.pose.is_moved(self.cache)):
            self.vel.set(self.pose.vec.path_from(self.cache.vec))
            self.cache.set(self.pose)

    def gap_to(self,subject):
        return self.pose.gap_to(subject.pose)

    def path_to(self,subject):
        return self.pose.vec.path_to(subject.pose.vec)

    def path_from(self,subject):
        return self.pose.vec.path_from(subject.pose.vec)

    def para(self,opposite_subject):
        return self.vel.para(opposite_subject.vel)

    def perp(self,opposite_subject):
        return self.vel.perp(opposite_subject.vel)
