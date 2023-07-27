from Queue import Queue
from rospy import Publisher
from math import sqrt
from datetime import datetime

def calculate_vel(p1, p2, rate):
    del_x = p2[0] - p1[0]
    del_y = p2[1] - p1[1]
    dis = sqrt(del_x ** 2+ del_y ** 2)
    return dis*rate # multiply by frequency of program

def shift_queue(q, est_vel, shift_num, rate):
    points = list(q.queue)
    #print(points)
    vel_est = []
    for i in range(q.qsize() - 1):
        vel_est.append(calculate_vel(points[i], points[i + 1], rate))
    avg_vel_est = sum(vel_est)/(q.qsize() - 1)
    est_vel.publish(avg_vel_est)
    for i in range(shift_num):
        robot_pos = q.get()

def generate_cur_timestamp():
    timestamp = datetime.now()
    formatted_timestamp = timestamp.strftime("%M:%S.%f")
    return formatted_timestamp

def time_difference(t1, t2):
    date_format = "%M:%S.%f"
    prev = datetime.strptime(t1, date_format)
    now = datetime.strptime(t2, date_format)
    return (now - prev).total_seconds() * 1000

def find_operation_time(t1, t2):
    t1 = t2
    t2 = generate_cur_timestamp()
    return t1, t2, time_difference(t1, t2)

