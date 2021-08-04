#!/usr/bin/env python
# -*- coding:utf-8 -*-

import rospy
import time
import pickle
import sys
import numny as np

#from autoware_msgs.msg import DetectedObjectArray
from geometry_msgs.msg import PoseStamped, Point32, Twist
from sensor_msgs.msg import PointCloud, ChannelFloat32
from std_msgs.msg import Int16, Bool
import tf

class AutoIntervention():

    def __init__(self, load_data_file, intervention, out_data_file):
        self.intervention = intervention
        self.intervened = False
        self.current_data_index = 0
        self.current_pose = None
        self.time_step_data = []
        self.out_data_file

        with open(load_data_file, 'rb') as f:
            self.data = pickle.load(f)

        self.tf_listener = tf.TransformListener()

        self.pub_twist = rospy.Publisher('/twist_cmd', Twist, queue_size=1)
        self.pub_intervention = rospy.Publisher('/is_intervened', Bool, queue_size=1)
        self.sub_twist = rospy.Subscriber('/twist_cmd_autoware', Twist, self.twistCb)
        self.sub_current_scenario = rospy.Subscriber('/current_scenario', Int16, self.currentScenarioCb)
        rospy.Timer(rospy.Duration(0.5), self.storeData)


    # def quatToYaw(orientation):
    #     euler = tf.transformations.euler_from_quaternion((orientation.x, orientation.y, orientation.z, orientation.w))
    #     return euler[2]

    def currentScenarioCb(self, msg):
        self.current_data_index = msg.data

    def twistCb(self, msg):

        if not self.intervention:
            return

        intervention_thres = 10.0 / 3.6
        carla_speed = self.data[self.current_data_index].get('waypoint')[4]
        autoware_speed = msg.twist.linear.x
        out_twist = Twist()

        if carla_speed - intervention_thres < carla_speed < autoware_speed + intervention_thres:
            out_twist = msg
            self.pub_intervention.publish(Bool(data=False))
        else:
            out_twist = msg
            out_twist.linear.x = carla_speed
            self.pub_intervention.publish(Bool(data=True))

        self.pub_twist.publish(out_twist)
        

if __name__ == '__main__':
    auto_intervention = AutoIntervention(sys.args[1], sys.args[2], sys.args[3])
