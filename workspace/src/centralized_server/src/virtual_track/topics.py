import rospy
import numpy as np
from geometry_msgs.msg import PoseStamped, Point, Quaternion
from std_msgs.msg import Float64

class MotiveSub:
    '''
    Subscribes to the Optitrack Pose data via the vrpn client
    '''

    def __init__(self, car):
        self.position = Point(2.0, 0, 0)
        self.orientation = Quaternion()
        self.motive_sub = rospy.Subscriber('vrpn_client_node/' + car + '/pose', PoseStamped, self.callback)

    def callback(self, data):
        self.data = data
        self.orientation = data.pose.orientation
        self.position = data.pose.position

    def get_pose(self):
        x = self.position.x
        y = self.position.y
        return np.array([x, y])

    def get_orientation(self):
        return self.orientation

class EstVelSub:
    '''
    Subscribes to the estimated velocity, calculated with the Motive data
    '''

    def __init__(self):
        self.vel = 0.0
        self.est_vel_sub = rospy.Subscriber('/est_vel', Float64, self.callback)

    def callback(self, data):
        self.vel = data.data

    def get_velocity(self):
        return self.vel
