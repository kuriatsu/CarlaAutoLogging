#!/usr/bin/env python
# -*- coding:utf-8 -*-

import rospy
import time
import pickle
import sys
import numpy as np

from autoware_msgs.msg import VehicleCmd
from geometry_msgs.msg import PoseStamped, Point32, Twist
from sensor_msgs.msg import PointCloud, ChannelFloat32
from std_msgs.msg import Int16, Bool
import tf

class AutoIntervention():

    def __init__(self, load_data_file, intervention):
        self.intervention = Bool(intervention)
        self.intervened = False
        self.current_data_index = 0
        self.current_pose = None
        self.time_step_data = []

        with open(load_data_file, 'rb') as f:
            self.data = pickle.load(f)

        # self.tf_listener = tf.TransformListener()

        self.pub_twist = rospy.Publisher('/vehicle_cmd', VehicleCmd, queue_size=1)
        self.pub_intervention = rospy.Publisher('/is_intervened', Bool, queue_size=1)
        self.sub_twist = rospy.Subscriber('/vehicle_cmd_autoware', VehicleCmd, self.twistCb)
        self.sub_current_scenario = rospy.Subscriber('/current_scenario', Int16, self.currentScenarioCb)
        # rospy.Timer(rospy.Duration(0.5), self.storeData)


    # def quatToYaw(orientation):
    #     euler = tf.transformations.euler_from_quaternion((orientation.x, orientation.y, orientation.z, orientation.w))
    #     return euler[2]

    def currentScenarioCb(self, msg):
        self.current_data_index = msg.data

    def twistCb(self, msg):

        intervention_thres = 5.0 / 3.6
        carla_speed = self.data[self.current_data_index].get('waypoint')[4]
        autoware_speed = msg.twist_cmd.twist.linear.x
        out_twist = VehicleCmd()
        if (autoware_speed > carla_speed + intervention_thres) or (autoware_speed < 10.0/3.6) and self.intervention:
        # if carla_speed - intervention_thres < autoware_speed < carla_speed + intervention_thres:
            out_twist = msg
            out_twist.twist_cmd.twist.linear.x = carla_speed
            out_twist.twist_cmd.twist.angular.z *= 2 * carla_speed / autoware_speed
            self.pub_intervention.publish(Bool(data=True))
            print(self.data[self.current_data_index].get('speed_limit'), carla_speed, autoware_speed, 'true')
        else:
            out_twist = msg
            self.pub_intervention.publish(Bool(data=False))
            print(self.data[self.current_data_index].get('speed_limit'), carla_speed, autoware_speed, 'false')


        self.pub_twist.publish(out_twist)


if __name__ == '__main__':
    rospy.init_node('auto_intervention_node')
    auto_intervention = AutoIntervention(sys.argv[1], sys.argv[2])
    rospy.spin()
