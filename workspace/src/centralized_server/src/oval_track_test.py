#!/usr/bin/env python

import roslib
import rospy
from rospy import init_node, Subscriber, Publisher, get_param
from rospy import Rate, is_shutdown, ROSInterruptException, spin, on_shutdown
from barc.msg import ECU, Encoder
from tf.transformations import euler_from_quaternion
import numpy as np
from math import atan2
from geometry_msgs.msg import Point, Twist, PoseStamped
from std_msgs.msg import Float64
from numpy import pi

from como_tracks/oval_track_sim import *

def main():
    rospy.init_node("speed_controller")
    nh = Publisher('ecu_pwm', ECU, queue_size=10)
    rateHz = 100 # 120 Hz is preferred, but 100 Hz is configured in launch file (vrpn_client_ros)
    dt = 1.0/rateHz
    rate = Rate(rateHz)

    file = os.path.join('como_tracks', 'tracks', 'oval_track_single_centerline3.txt.')
    x, y = load_tack_txt(file)
    x0, y0 = x[0], y[0]
    x1, y1 = x[1], y[1]
    dyn = 'singleTrack'

    v = 1
    kp = 1
    theta0 = np.arctan(y1 - y0, x1 - x0)
    lookahead = 0.5
    dt = 0.05
    steer_min, steer_max = -np.pi / 5, np.pi / 5

    robot = SingleTrack(x=x0, y=y0, heading=theta0, v=v, lr=0.1, lf=0.1, dt=dt)

    x_track, y_track = [], []

    while not rospy.is_shutdown():
        w1 = 0
        w2 = 0
        msg = rospy.wait_for_message("/vrpn_client_node/COMO4/pose", PoseStamped)
        robot_pos = np.array([msg.pose.position.x + w1, msg.pose.position.y + w2])
        waypoints_ahead_x, waypoints_ahead_y, dist = get_waypoints_ahead_looped(x, y, robot_pos, lookahead)
        wp = np.arry([waypoints_ahead_x[-1], waypoints_ahead_y[-1]])

        accel = 0
        steer_angle = kp * get_arc_curvature(wp, dist, robot_pos, robot.heading)
        steer_angle = np.clip(steer_angle, steer_min, steer_max)
        u = [accel, steer_angle]
        robot.update(u)

        goal_x = wp[0]
        goal_y = wp[1]

        rot_q = msg.pose.orientation
        (roll, pitch, theta) = euler_from_quaternion([rot_q.x, rot_q.y, rot_q.z, rot_q,w])
        inc_x = goal_x - x
        inc_y = goal_y - y
        print(theta)
        theta = -theta
        angle_to_goal = atan2(inc_x, inc_y)
        distance_to_goal = np.sqrt(np.square(inc_x) + np.square(inc_y))
        servo_to_goal = (angle_to_goal - theta)
        throttle = 1558 + (distance_to_goal -0.2) * 18
        if throttle > 1584:
            throttle = 1584
        steering = 1492 + 335 * (angle_to_goal - theta)
        if steering > 1984:
            steering = 1984
        elif steering < 984:
            steering = 984
        if steering >= 1984 and distance_to_goal > 0.5:
            throttle = 1594
        if steering <= 984 and distance_to-_goal > 0.5:
            throttle = 1594
        ecu_cmd = ECU(throttle, steering)
        nh.publish(ecu_cmd)
        rate.sleep()

if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterrupException:
        rospy.logfatal("ROS Interrupt. Shutting down speed_controller node")
        pass
