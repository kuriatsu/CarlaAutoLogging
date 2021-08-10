#!/usr/bin/env python
# -*- coding:utf-8 -*-

import rospy
import time
import pickle
import sys
import numpy as np
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Int32, Bool
from autoware_msgs.msg import VehicleCmd
from sensor_msgs.msg import PointCloud2, PointField
import sensor_msgs.point_cloud2 as pc2
import tf

class SaveRosData():
    def __init__(self, load_data_file, out_data_file):

        with open(load_data_file, 'rb') as f:
            self.data = pickle.load(f)

        self.current_data_index = 0
        self.current_pose = None
        self.current_twist = None
        self.is_intervened = False
        self.is_collided = False

        self.out_data_file = out_data_file
        self.time_step_data = []

        self.sub_current_scenario = rospy.Subscriber('/current_scenario', Int32, self.currentScenarioCb)
        self.sub_current_pose = rospy.Subscriber('/current_pose', PoseStamped, self.currentPoseCb)
        self.sub_vehicle_cmd = rospy.Subscriber('/vehicle_cmd', VehicleCmd, self.vehicleCmdCb)
        self.sub_intervention = rospy.Subscriber('/is_intervened', Bool, self.interventionCb)
        self.sub_intervention = rospy.Subscriber('/is_collided', Bool, self.collisionCb)
        rospy.Timer(rospy.Duration(0.5), self.storeData)


    def currentScenarioCb(self, msg):
        self.current_data_index = msg.data


    def currentPoseCb(self, msg):
        self.current_pose = msg.pose


    def interventionCb(self, msg):
        self.is_intervened = msg.data


    def collisionCb(self, msg):
        self.is_collided = msg.data


    def vehicleCmdCb(self, msg):
        self.current_twist = msg.twist_cmd.twist


    def storeData(self, event):
        if self.current_twist is None or self.current_pose is None:
            return

        actors_data = {}
        step_data = {}
        actors_data['ego_vehicle'] = {
                'type' : self.data[0].get('type'),
                'pose' : [self.current_pose.position.x, self.current_pose.position.y, self.current_pose.position.y, self.quatToYaw(self.current_pose.orientation)],
                'speed' : self.current_twist.linear.x,
                'size' : self.data[0].get('size'),
                }

        for id, actor in self.data[self.current_data_index].get('actors').items():
            actors_data[id] = actor

        step_data = {
            'time' : rospy.get_time(),
            'mileage' : self.current_data_index * 0.5,
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


if __name__ == '__main__':
    try:
        print('start')
        rospy.init_node('save_ros_data_node')
        save_ros_data = SaveRosData(sys.argv[1], sys.argv[2])
        rospy.spin()
    finally:
    # except KeyboardInterrupt:
        save_ros_data.saveData()
