#!/usr/bin/env python
import rospy
from circleFit import fit_circle
import numpy as np
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2
from scipy.signal import find_peaks
from intro_navigation.msg import ObstaclePositionScan
import matplotlib.pyplot as plt
from circleFit import points_circle


def get_convex_index(d_grad_angle, small_delta=3e-1):
    return d_grad_angle < -small_delta


def get_small_magnitude_index(grad_magnitude, big_delta=1e1):
    return (grad_magnitude > -big_delta) & (grad_magnitude < big_delta)


def obstacles_circles(x, y, grad_magnitude, convex_index, inside_object_index, min_points=10, big_delta=1e1):
    # init no point accepted_index
    peaks, _ = find_peaks(grad_magnitude, height=big_delta)
    accepted_index = np.full(x.shape, False, dtype=bool)
    accepted_peaks = list()
    index = convex_index & inside_object_index
    for i in range(len(peaks)-1):
        start_peak = peaks[i]
        end_peak = peaks[i+1]
        if np.count_nonzero(index[start_peak:end_peak]) >= min_points:
            accepted_index[start_peak:end_peak] = index[start_peak:end_peak]
            accepted_peaks.append((start_peak, end_peak))
    circles = list()
    for start, end in accepted_peaks:
        object_x = x[start:end]
        object_y = y[start:end]
        object_x = object_x[accepted_index[start:end]]
        object_y = object_y[accepted_index[start:end]]
        xc, yc, rc = fit_circle(object_x, object_y)
        circles.append((xc, yc, rc))
    return circles


def points_callback(cloud_points_msg):
    integration_step = 0.00577401509508
    points = pc2.read_points_list(cloud_points_msg)
    x = np.asarray([val.x for val in points])
    y = np.asarray([val.y for val in points])
    z = np.asarray([val.z for val in points])
    dx = np.gradient(x, integration_step)
    dy = np.gradient(y, integration_step)
    grad_magnitude = np.sqrt(np.square(dx) + np.square(dy))
    # in objects the angle is decreasing/increasing if convex/concave
    # negative is for objects and positive for the room near constant for walls
    # inside the objects the magnitude is relatively low
    grad_angle = np.arctan2(dy, dx)
    # the derivative of the gradient angle tells if it's a wall, convex, concave
    d_grad_angle = np.gradient(grad_angle, integration_step)
    try:
        circles = obstacles_circles(x, y, grad_magnitude,
                                    get_convex_index(d_grad_angle),
                                    get_small_magnitude_index(grad_magnitude))
        out_msg = ObstaclePositionScan()
        out_msg.x = [val[0] for val in circles]
        out_msg.y = [val[1] for val in circles]
        out_msg.r = [val[2] for val in circles]
        obstacles_publisher.publish(out_msg)

        # plt.clf()
        # plt.scatter(x, y, c="blue", s=0.1)
        # for i in range(len(out_msg.x)):
        #   xc, yc = points_circle(out_msg.x[i], out_msg.y[i], out_msg.r[i])
        #   plt.scatter(xc, yc, c="red", s=0.1)
        # plt.draw()
        # plt.pause(0.001)
    except np.linalg.LinAlgError:
        rospy.logwarn("Singular matrix encountered")


if __name__ == "__main__":
    rospy.init_node('obstacles_position_node')
    scan_subscriber = rospy.Subscriber('/intro_navigation/scan_cloud', PointCloud2, points_callback)
    obstacles_publisher = rospy.Publisher('/intro_navigation/obstacles_centers', ObstaclePositionScan, queue_size=100)
    rospy.loginfo("Node started -- reading cloudPoint2 from /intro_navigation/scan_cloud"
                  + " and writing ObstaclePositionScan into /intro_navigation/obstacles_centers wrt map frame")
    rospy.spin()
