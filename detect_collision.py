#!/usr/bin/env python
# -*- coding:utf-8 -*-

import rospy
import time
import pickle
import sys
import numpy as np

import tf
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2
from std_msgs.msg import Bool, Float32MultiArray, String

class DetectCollision():

    def __init__(self):
        self.vehicle_size = None

        self.pub_collision = rospy.Publisher('/is_collided', Bool, queue_size=1)
        self.pub_string = rospy.Publisher('/collision_message', String, queue_size=1)
        self.sub_points = rospy.Subscriber('/points_no_ground', PointCloud2, self.pointsCb)
        self.sub_vehicle_size = rospy.Subscriber('/ego_vehicle/size', Float32MultiArray, self.vehicleSizeCb)


    def pointsCb(self, msg):
        judge_points = []
        for point in pc2.read_points(msg, field_names=('x', 'y', 'z', 'rgb'), skip_nans=True):
            distance = (point[0]**2 + point[1]**2)**0.5
            if distance < 3.0:
                judge_points.append(point)

        if not judge_points:
            self.pub_collision.publish(Bool(data=False))
            self.pub_string.publish(String(data=''))
            return

        for point in judge_points:
            if -self.vehicle_size[0] < point[0] < self.vehicle_size[0] or -self.vehicle_size[1] < point[1] < self.vehicle_size[1]:
                self.pub_collision.publish(Bool(data=True))
                self.pub_string.publish(String(data='Collide'))
                return

        self.pub_collision.publish(Bool(data=False))
        self.pub_string.publish(String(data=''))


    def vehicleSizeCb(self, msg):
        self.vehicle_size = msg.data


if __name__ == '__main__':
    rospy.init_node('detect_collision_node')
    save_ros_data = DetectCollision()
    rospy.spin()
