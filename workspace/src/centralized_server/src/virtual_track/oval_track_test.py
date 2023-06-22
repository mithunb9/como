#!/usr/bin/env python

import roslib
import rospy
import rospkg
from rospy import init_node, Subscriber, Publisher, get_param
from rospy import Rate, is_shutdown, ROSInterruptException, spin, on_shutdown
from barc.msg import ECU, Encoder
from tf.transformations import euler_from_quaternion
import numpy as np
from math import atan2
from geometry_msgs.msg import Point, Twist, PoseStamped
from std_msgs.msg import Float64
from numpy import pi

from como_tracks.oval_track_sim import * 

def bound_servo_angle(servo_angle):
    if servo_angle > 0:
        servo_angle -= np.pi * 2
    else:
        servo_angle += np.pi * 2
    return servo_angle

def main():
    rospy.init_node("speed_controller")
    nh = Publisher('ecu_pwm', ECU, queue_size=10)
    rateHz = 100 # 120 Hz is preferred, but 100 Hz is configured in launch file (vrpn_client_ros)
    dt = 1.0/rateHz
    rate = Rate(rateHz)

    rospack = rospkg.RosPack()
    package_path = rospack.get_path('test_launches')
    file = os.path.join(package_path, 'scripts', 'virtual_track', 'como_tracks', 'tracks', 'oval_track_single_centerline4.txt')
    print(file)
    x, y = load_tack_txt(file)
    x0, y0 = x[0], y[0]
    x1, y1 = x[1], y[1]
    dyn = 'singleTrack'

    v = 1
    kp = 1
    theta0 = np.arctan2(y1 - y0, x1 - x0)
    lookahead = 0.3
    dt = 0.05
    steer_min, steer_max = -np.pi / 4, np.pi / 4

    robot = SingleTrack(x=x0, y=y0, heading=theta0, v=v, lr=0.1, lf=0.1, dt=dt)

    x_track, y_track = [], []

    while not rospy.is_shutdown():
        msg = rospy.wait_for_message("/vrpn_client_node/COMO4/pose", PoseStamped)
        robot_pos = np.array([msg.pose.position.x, msg.pose.position.y])
        waypoints_ahead_x, waypoints_ahead_y, dist = get_waypoints_ahead_looped(x, y, robot_pos, lookahead)
        wp = np.array([waypoints_ahead_x[-1], waypoints_ahead_y[-1]])
        
        x_track.append(robot_pos[0])
        y_track.append(robot_pos[1])

        goal_x = wp[0]
        goal_y = wp[1]
        print("Pos", robot_pos[0], robot_pos[1])
        rot_q = msg.pose.orientation
        (roll, pitch, theta) = euler_from_quaternion([rot_q.x, rot_q.y, rot_q.z, rot_q.w])
        inc_x = goal_x - robot_pos[0]
        inc_y = goal_y - robot_pos[1]
        print("Way", goal_x, goal_y)
        print("Dis", inc_x, inc_y)
        #theta = -theta
        angle_to_goal = atan2(inc_y, inc_x) - np.pi/2
        print("Ang", theta, angle_to_goal)
        distance_to_goal = np.sqrt(np.square(inc_x) + np.square(inc_y))
        servo_to_goal = (angle_to_goal - theta)
        servo_to_goal = -servo_to_goal
        if abs(servo_to_goal) > np.pi:
            servo_to_goal = bound_servo_angle(servo_to_goal)
        print(servo_to_goal)
        #throttle = 1558 + (distance_to_goal) * 50
        throttle = 1585
        #if throttle > 1584:
        #    throttle = 1584
        steering = 1524 + 250 * servo_to_goal
        if steering > 1824:
            steering = 1824
        elif steering < 1224:
            steering = 1224
        print(throttle, steering)
        #if steering >= 1984 and distance_to_goal > 0.5:
        #    throttle = 1894
        #if steering <= 984 and distance_to_goal > 0.5:
        #    throttle = 1894
        ecu_cmd = ECU(throttle, steering)
        nh.publish(ecu_cmd)
        rate.sleep()

if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        rospy.logfatal("ROS Interrupt. Shutting down speed_controller node")
        pass
