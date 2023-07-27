#!/usr/bin/env python

import roslib
import rospy
from rospy import init_node, Subscriber, Publisher, get_param
from rospy import Rate, is_shutdown, ROSInterruptException, spin, on_shutdown
from barc.msg import ECU, mod_ECU, Encoder
from tf.transformations import euler_from_quaternion
import numpy as np
from math import atan2, sqrt
from geometry_msgs.msg import Point, Twist, PoseStamped
from std_msgs.msg import Float64, String
from numpy import pi
from como_tracks.oval_track_sim import * 
from post_processing import *
from file_handling import *
from Queue import Queue
from real_time_operations import *
from topics import *

def bound_servo_angle(servo_angle):
    if servo_angle > 0:
        servo_angle -= np.pi * 2
    else:
        servo_angle += np.pi * 2
    return servo_angle

x_track, y_track, heading_track, waypoint_x_track, waypoint_y_track = [], [], [], [], []
package_path = find_package_path('test_launches')

def main():
    rospy.init_node("speed_controller")
    #nh = Publisher('ecu_pwm', ECU, queue_size=10)
    nh = Publisher('ecu', mod_ECU, queue_size=10)
    est_vel = Publisher('est_vel', Float64, queue_size=10)
    rateHz = 100 # 120 Hz is preferred, but 100 Hz is configured in launch file (vrpn_client_ros)
    dt = 1.0/rateHz
    rate = Rate(rateHz)

    file = os.path.join(package_path, 'scripts', 'virtual_track', 'como_tracks', 'tracks', 'oval_track_single_centerline5.txt')
    #print(file)
    global x, y
    x, y = load_tack_txt(file)
    x0, y0 = x[0], y[0]
    x1, y1 = x[1], y[1]
    dyn = 'singleTrack'

    v = 1
    kp = 1
    theta0 = np.arctan2(y1 - y0, x1 - x0)
    lookahead = 0.5
    dt = 0.05
    steer_min, steer_max = -np.pi / 4, np.pi / 4

    #robot = SingleTrack(x=x0, y=y0, heading=theta0, v=v, lr=0.1, lf=0.1, dt=dt)
    
    motive_sub = MotiveSub("COMO4")
    est_vel_sub = EstVelSub()

    global pos_queue
    pos_queue = Queue(maxsize=10)
    c_idx = 0
    
    t1 = generate_cur_timestamp()
    t2 = t1

    while not rospy.is_shutdown():
        #msg = rospy.wait_for_message("/vrpn_client_node/COMO4/pose", PoseStamped)
        #robot_pos = np.array([msg.pose.position.x, msg.pose.position.y])
        robot_pos = motive_sub.get_pose()
       
        t1, t2, diff = find_operation_time(t1, t2)
        #print("Optitrack Data", diff)
        pos_queue.put(robot_pos)
        if pos_queue.full():
            shift_queue(pos_queue, est_vel, 2, rateHz)
        t1, t2, diff = find_operation_time(t1, t2)
        #print("Shift Queue", diff)
        #waypoints_ahead_x, waypoints_ahead_y, dist = get_waypoints_ahead_looped(x, y, robot_pos, lookahead)
        waypoints_ahead_x, waypoints_ahead_y, dist, close_idx = get_waypoints_heuristic(x, y, robot_pos, lookahead, c_idx)
        c_idx = close_idx
        wp = np.array([waypoints_ahead_x[-1], waypoints_ahead_y[-1]])
        t1, t2, diff = find_operation_time(t1, t2)
        #print("Find Waypoints", diff)
        x_track.append(robot_pos[0])
        y_track.append(robot_pos[1])

        goal_x = wp[0]
        goal_y = wp[1]
        #waypoint_x_track.append(goal_x)
        #waypoint_y_track.append(goal_y)
        
        #print("Pos", robot_pos[0], robot_pos[1])
        rot_q = motive_sub.get_orientation()
        (roll, pitch, theta) = euler_from_quaternion([rot_q.x, rot_q.y, rot_q.z, rot_q.w])
        t1, t2, diff = find_operation_time(t1, t2)
        #print("Calculate yaw", diff)
        inc_x = goal_x - robot_pos[0]
        inc_y = goal_y - robot_pos[1]
        #print("Way", goal_x, goal_y)
        #print("Dis", inc_x, inc_y)
        #heading_track.append(theta)

        angle_to_goal = atan2(inc_y, inc_x) - np.pi/2
        #print("Ang", theta, angle_to_goal)
        distance_to_goal = np.sqrt(np.square(inc_x) + np.square(inc_y))
        servo_to_goal = (theta - angle_to_goal)
        t1, t2, diff = find_operation_time(t1, t2)
        #print("Calculate distance/angle", diff)
        if abs(servo_to_goal) > np.pi:
            servo_to_goal = bound_servo_angle(servo_to_goal)
        #print(servo_to_goal)
        t1, t2, diff = find_operation_time(t1, t2)
        #print("Bound servo angle", diff)
        #throttle = 1630
        motor = 7.75
        motor_gain = 1.0
       	servo_gain = 1
        
        desired_speed = 1.8
        cur_speed = est_vel_sub.get_velocity()

        if abs(servo_to_goal) > 0.05: 
            servo_gain = (servo_to_goal/5.25 + 1)

        #if abs(servo_to_goal) > 0.2:
        #    motor = 6.75

        motor_gain = (desired_speed - cur_speed)/2 + 1

        t1, t2, diff = find_operation_time(t1, t2)
        #print("Adjust steering gains", diff)
        #print(motor, servo_gain)
        '''
        steering = 1524 + 250 * servo_to_goal
        if steering > 1824:
            steering = 1824
        elif steering < 1224:
            steering = 1224
        print(throttle, steering)
        '''
        servo_to_goal = np.clip(servo_to_goal, steer_min, steer_max)
        servo_to_goal += np.pi/2
        t1, t2, diff = find_operation_time(t1, t2)
        #print("Reorient servo", diff)
        ecu_cmd = mod_ECU(motor, servo_to_goal, motor_gain, servo_gain)
        #ecu_cmd = ECU(throttle, steering)
        nh.publish(ecu_cmd)
        t1, t2, diff = find_operation_time(t1, t2)
        #print("Publish command", diff)
        rate.sleep()
        t1, t2, diff = find_operation_time(t1, t2)
        #print("Rate sleep", diff)

def shutdown_handler():
    timestamp = generate_timestamp()
    param_log = rospy.get_param('/oval_track_follower/log')
    param_plot = rospy.get_param('/oval_track_follower/plot')
    if param_log:
        save_track(package_path, timestamp, x_track, y_track, heading_track, waypoint_x_track, waypoint_y_track)
    if param_plot:
        plot_track(x, y, x_track, y_track, timestamp, package_path)

if __name__ == "__main__":
    try:
        rospy.on_shutdown(shutdown_handler)
        main()
    except rospy.ROSInterruptException:
        rospy.logfatal("ROS Interrupt. Shutting down speed_controller node")
        pass
