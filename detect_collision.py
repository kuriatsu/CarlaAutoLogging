#!/usr/bin/env python
# -*- coding:utf-8 -*-

import rospy
import time
import pickle
import sys
import numpy as np

import tf
from geometry_msgs.msg import PoseStamped, Point
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2
from std_msgs.msg import Bool

class DetectCollision():

    def __init__(self):
        self.vehicle_size = None

        self.sub_points = rospy.Subscriber('/points_no_ground', PointCloud2, self.pointsCb)
        self.sub_vehicle_size = rospy.Subscriber('/ego_vehicle/size', Point, self.vehicleSizeCb)
        self.pub_collision = rospy.Publisher('/is_collided', Bool, queue_size=1)


    def pointsCb(self, msg):
        judge_points = []
        for point in pc2.read_points(msg, field_names=('x', 'y', 'z', 'rgb'), skip_nans=True):
            distance = (point[0]**2 + point[1]**2)**0.5
            if distance < 3.0:
                judge_points.append(point)

        if not judge_points:
            self.pub_collision.publish(Bool(data=False))
            return

        for point in judge_points:
            if -self.vehicle_size.x < point[0] < self.vehicle_size.x or -self.vehicle_size.y < point[1] < self.vehicle_size.y:
                self.pub_collision.publish(Bool(data=True))
                return

        self.pub_collision.publish(Bool(data=False))


    def vehicleSizeCb(self, msg):
        self.vehicle_size = msg


if __name__ == '__main__':
    rospy.init_node('detect_collision_node')
    save_ros_data = DetectCollision()
    rospy.spin()
