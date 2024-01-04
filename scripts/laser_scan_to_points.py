#!/usr/bin/env python
import rospy
import numpy as np
import matplotlib.pyplot as plt
import math
from sensor_msgs.msg import LaserScan
from sklearn.cluster import DBSCAN


class laser_scan_to_points:
    def __init__(self):
        self.scan_sub = rospy.Subscriber("scan", LaserScan, self.scanCallback, queue_size=1)
        self.angle_min = 3.1372294426
        self.angle_increment = -0.00436332309619
        plt.ion()

    def scanCallback(self, msg):
        tic = rospy.Time.now().to_sec()
        ranges = msg.ranges
        num_pts = len(ranges)
        pts = []
        
        for i in range(num_pts):
            if ranges[i] > 5 or math.isnan(ranges[i]):
                pass
            else:
                angle = self.angle_min + float(i) * self.angle_increment
                x = float(ranges[i] * math.cos(angle))
                y = float(ranges[i] * math.sin(angle))
                pts.append([x, y])

        clustering = DBSCAN(eps=0.5, min_samples=5).fit(pts)

        _x = [pt[0] for pt in pts]
        _y = [pt[1] for pt in pts]

        plt.clf()
        plt.scatter(_x, _y, c=clustering.labels_, cmap="Paired")
        plt.xlim([-5, 5])
        plt.ylim([-5, 5])
        plt.show()

        toc = rospy.Time.now().to_sec()
        rospy.loginfo("clustering runtime: %f", toc - tic)

        plt.pause(0.1)


if __name__=='__main__':
    rospy.init_node("laser_scan_to_points")
    laser_scan_to_points = laser_scan_to_points()
    rospy.loginfo("performing laser scan clustering.")
    rospy.spin()
