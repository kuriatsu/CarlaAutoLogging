#!/usr/bin/env python
# -*- coding:utf-8 -*-

import rospy
import time
import pickle
import sys
import numpy as np
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Int32, Bool, Float32MultiArray, String
from autoware_msgs.msg import VehicleCmd, DetectedObjectArray, DetectedObject
from sensor_msgs.msg import PointCloud2, PointField
import sensor_msgs.point_cloud2 as pc2
import tf

class SaveRosData():
    def __init__(self, out_data_file):

        self.objects = None
        self.current_pose = None
        self.current_twist = None
        self.is_intervened = False
        self.is_collided = False
        self.ego_vehicle_size = None
        self.ego_vehicle_type = None

        self.out_data_file = out_data_file
        self.time_step_data = []

        self.sub_object = rospy.Subscriber('/detection/contour_tracker/objects', DetectedObjectArray, self.objectCb)
        self.sub_current_pose = rospy.Subscriber('/current_pose', PoseStamped, self.currentPoseCb)
        self.sub_vehicle_cmd = rospy.Subscriber('/vehicle_cmd', VehicleCmd, self.vehicleCmdCb)
        self.sub_intervention = rospy.Subscriber('/is_intervened', Bool, self.interventionCb)
        self.sub_collision = rospy.Subscriber('/is_collided', Bool, self.collisionCb)
        self.sub_ego_vehicle_size = rospy.Subscriber('/ego_vehicle/size', Float32MultiArray, self.sizeCb)
        self.sub_ego_vehicle_type = rospy.Subscriber('/ego_vehicle/type', String, self.typeCb)

        rospy.Timer(rospy.Duration(0.5), self.timerCb)


    def timerCb(self, event):
        if self.current_twist is None or self.current_pose is None or self.objects is None:
            return

        actors_data = {}
        step_data = {}
        actors_data['ego_vehicle'] = {
                'type' : self.ego_vehicle_type,
                'pose' : [self.current_pose.position.x, self.current_pose.position.y, self.current_pose.position.y, self.quatToYaw(self.current_pose.orientation)],
                'speed' : self.current_twist.linear.x,
                'size' : self.ego_vehicle_size,
                }

        for actor in self.objects:
            actors_data[actor.id] = {
            'type' : actor.label,
            'pose' : [actor.pose.position.x, actor.pose.position.y, actor.pose.position.z, self.quatToYaw(actor.pose.orientation)],
            'speed' : actor.velocity.linear.x,
            'size' : [actor.dimensions.x, actor.dimensions.y, actor.dimensions.z],
            }

        step_data = {
            'time' : rospy.get_time(),
            'actors' : actors_data,
            'intervention' : self.is_intervened,
            'collision' : self.is_collided,
            }
        self.time_step_data.append(step_data)


    def quatToYaw(self, quat):
        euler = tf.transformations.euler_from_quaternion((quat.x, quat.y, quat.z, quat.w))
        return euler[2]


    def saveData(self):
        print('save_ros_data')
        with open(self.out_data_file, 'wb') as f:
            pickle.dump(self.time_step_data, f)


    def objectCb(self, msg):
        self.objects = msg


    def currentPoseCb(self, msg):
        self.current_pose = msg.pose


    def interventionCb(self, msg):
        self.is_intervened = msg.data


    def collisionCb(self, msg):
        self.is_collided = msg.data


    def vehicleCmdCb(self, msg):
        self.current_twist = msg.twist_cmd.twist


    def sizeCb(self, msg):
        self.ego_vehicle_size = msg.data


    def typeCb(self, msg):
        self.ego_vehicle_type = msg.data


if __name__ == '__main__':
    # print('start')
    # rospy.init_node('save_ros_data_node')
    # save_ros_data = SaveRosData(sys.argv[1])
    # rospy.spin()
    # save_ros_data.saveData()
    try:
        print('start')
        rospy.init_node('save_ros_data_node')
        save_ros_data = SaveRosData(sys.argv[1])
        rospy.spin()
    finally:
        save_ros_data.saveData()
