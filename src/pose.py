from vector import Vector

class Pose:
    # attributes
    # - vec : Vector
    # - yaw : float
    # properties
    # - x : float
    # - y : float
    # methods
    # - is_moved(pose : Pose)
    # - is_moved(pose : Pose, threshold : float)
    # aliases
    # - gap_to -> Vector

    def __init__(self,arg1=None,arg2=None,arg3=None):
        if(arg1 is None):
            # Pose()
            self.vec = Vector()
            self.yaw = 0.0
        elif(isinstance(arg1,Vector)):
            # Pose(Vector,yaw)
            self.vec = arg1
            self.yaw = arg2
        else:
            # Pose(x,y,yaw)
            self.vec = Vector(arg1,arg2)
            self.yaw = arg3

    def set(self,pose):
        if(isinstance(pose,Pose)):
            # if pose is Pose that I defined
            self.vec.set(pose.x, pose.y)
            self.yaw = pose.yaw
        else:
            # if pose is geometry_msgs.Pose
            # why did I do such a messy naming??? :(
            self.vec.set(pose.position.x, pose.position.y)
            euler = tf.transformations.euler_from_quaternion((
                pose.orientation.x,
                pose.orientation.y,
                pose.orientation.z,
                pose.orientation.w
            ))
            self.yaw = euler[2]

    @property
    def x(self):
        return self.vec.x
    @property
    def y(self):
        return self.vec.y

    def gap_to(self,pose):
        return self.pose.vec.norm(pose.vec)

    def is_moved(self,pose,thre=None):
        if(thre in None):
            return self.gap_to(pose) > 0.5
        else:
            return self.gap_to(pose) > thre
