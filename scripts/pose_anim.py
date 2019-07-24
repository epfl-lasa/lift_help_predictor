#!/usr/bin/env python
# -*- coding: utf-8 -*-

import numpy as np
import matplotlib.pyplot as plt
import mpl_toolkits.mplot3d.axes3d as p3
import matplotlib.animation as animation
import pandas 
import rospy
from rospkg import RosPack
import os

CSV_DIR_PATH = RosPack().get_path('lift_help_predictor') + '/data/csv'
_CONNECTION = [
    [0, 1], [1, 2], [2, 3], [0, 4], [4, 5], [5, 6], [0, 7], [7, 8],
    [8, 9], [9, 10], [8, 11], [11, 12], [12, 13], [8, 14], [14, 15],
    [15, 16]]

def main():
    rospy.init_node("pose_anim", anonymous=True)
    bag_path = rospy.get_param("~bag_path")
    rate_coef = rospy.get_param("~rate")

    csv_file_name = bag_path.split(".bag")[0] + ".csv"
    csv_path = os.path.join(CSV_DIR_PATH, csv_file_name)

    data = parse_data(csv_path)

    pose = data[0]
    assert (pose.ndim == 2)
    assert (pose.shape[0] == 3)
    fig = plt.figure()
    ax = fig.gca(projection='3d')

    lines = []
    dots = []
    for c in _CONNECTION:
        col = '#%02x%02x%02x' % joint_color(c[0])
        lines.append(
            ax.plot([pose[0, c[0]], pose[0, c[1]]],
                    [pose[1, c[0]], pose[1, c[1]]],
                    [pose[2, c[0]], pose[2, c[1]]], c=col)
        )

    for j in range(pose.shape[1]):
        col = '#%02x%02x%02x' % joint_color(j)
        dots.append(
            ax.scatter(pose[0, j], pose[1, j], pose[2, j],
                       c=col, marker='o', edgecolor=col)
        )

    smallest = pose.min()
    largest = pose.max()
    ax.set_xlim3d(smallest, largest)
    ax.set_ylim3d(smallest, largest)
    ax.set_zlim3d(smallest, largest)

    ani = animation.FuncAnimation(fig, update_data, len(data), fargs=(data, lines, dots),
                                       interval=33.33/rate_coef, blit=False, repeat=True)

    plt.show()

def joint_color(j):
    """
    TODO: 'j' shadows name 'j' from outer scope
    """
    colors = [(0, 0, 0), (255, 0, 255), (0, 0, 255),
                (0, 255, 255), (255, 0, 0), (0, 255, 0)]
    _c = 0
    if j in range(1, 4):
        _c = 1
    if j in range(4, 7):
        _c = 2
    if j in range(9, 11):
        _c = 3
    if j in range(11, 14):
        _c = 4
    if j in range(14, 17):
        _c = 5
    return colors[_c]

    pose = data[0]

def update_data(num, data, lines, dots): 
    pose = data[num]
    for line_num in range(0, len(lines)): 
        line = lines[line_num][0]
        c = _CONNECTION[line_num]
        line.set_data([pose[0, c[0]], pose[0, c[1]]], [pose[1, c[0]], pose[1, c[1]]])
        line.set_3d_properties([pose[2, c[0]], pose[2, c[1]]])

    for dot_num in range(0, len(dots)):
        dot = dots[dot_num]
        dot.set_offsets( [pose[0, dot_num], pose[1, dot_num]] )
        dot.set_3d_properties(pose[2, dot_num], zdir='z')

def parse_data(csv_file_path):  
    data = []
    df = pandas.read_csv(csv_file_path)
    matrix = df.as_matrix()
    for index in range(0, matrix.shape[0]):
        data.append([[], [], []])
        ucl_data = matrix[index,1:52]
        for joint_start_index in range(0, ucl_data.size, 3): 
            for offset in range(0,3):
                data[index][offset].append(ucl_data[joint_start_index+offset])
            
    data = np.array(data)
    return data

# Run script
if __name__ == '__main__':
    import sys
    try:
        sys.exit(main())
    except rospy.ROSInterruptException:
        pass
