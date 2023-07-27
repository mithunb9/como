import os
import matplotlib.pyplot as plt
import csv

def save_track(package_path, timestamp, x, y, heading, waypoint_x, waypoint_y):
    headers = ['x', 'y', 'heading', 'waypoint_x', 'waypoint_y']
    data = []
    for i in range(len(x)):
        row = []
        row.append(x[i])
        row.append(y[i])
        row.append(heading[i])
        row.append(waypoint_x[i])
        row.append(waypoint_y[i])
        data.append(row)

    fileName = "otsl_pos" + timestamp + ".csv"
    path = os.path.join(package_path, 'data', 'logs', fileName)
    writer = csv.writer(open(path, 'w'), delimiter=',', lineterminator='\n')
    writer.writerow(headers)
    writer.writerows(data)

def plot_track(x, y, x_track, y_track, timestamp, package_path):
    plt.plot(x, y, "k", alpha=0.2)
    plt.plot(x_track, y_track, "m")
    plt.axis('equal')
    fileName = "otsl_overlay" + timestamp + ".png"
    path = os.path.join(package_path, 'data', 'figures', fileName)
    plt.savefig(path)

