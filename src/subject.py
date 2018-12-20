from vector import Vector
from pose   import Pose

class Subject:
    # attributes
    # - pose  : Pose
    # - cache : Pose
    # - vel   : Vector
    # methods
    # - set_pose(pose : Pose)
    # - calc_vel()
    # aliases
    # - gap_to  -> Pose
    # - path_to -> Vector

    def __init__(self):
        self.pose  = Pose()
        self.cache = None
        self.vel   = Vector()

    def set_pose(self,pose):
        self.pose.set(pose)
        self.calc_vel()

    def calc_vel(self):
        if(self.cache is None):
            self.cache = Pose(self.pos.x,self.pose.y)
        if(self.pos.is_moved(self.cache)):
            self.vel.set(self.pos.path_from(self.cache.x))
            self.cache.set(self.pos)

    def gap_to(subject):
        return self.pose.gap_to(subject.pose)

    def path_to(self,subject):
        return self.pose.vec.path_to(subject.pose.vec)
