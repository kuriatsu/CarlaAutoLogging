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
        self.sub_twist = rospy.Subscriber('/twist_cmd_autoware', Twist, self.twistCb)
        self.sub_current_pose = rospy.Subscriber('/current_pose', PoseStamped, self.currentPoseCb)
        rospy.Timer(rospy.Duration(0.5), self.storeData)


    def quatToYaw(orientation):
        euler = tf.transformations.euler_from_quaternion((orientation.x, orientation.y, orientation.z, orientation.w))
        return euler[2]


    def currentPoseCb(self, msg):
        self.current_pose = msg.pose
        current_wp = self.data[self.current_data_index].get('waypoint')
        next_wp = self.data[self.current_data_index+1].get('waypoint')
        dist_to_current_wp = (msg.pose.position.x - current_wp[0]) ** 2 + (msg.pose.position.y - current_wp[1]) ** 2
        dist_to_next_wp = (msg.pose.position.x - next_wp[0]) ** 2 + (msg.pose.position.y - next_wp[1]) ** 2

        if dist_to_current_wp > dist_to_next_wp:
            self.current_data_index += １

        if self.current_data_index == len(self.data - 1):
            self.saveData()
            exit()

    def twistCb(self, msg):

        if not self.intervention:
            return

        carla_speed = self.data[self.current_data_index].get('ego_vehicle').get('speed')
        autoware_speed = msg.twist.linear.x
        out_twist = Twist()

        if carla_speed < autoware_speed:
            out_twist = msg
            out_twist.linear.x = carla_speed
            self.intervened = True
        else:
            out_twist = msg
            self.intervened = False

        self.pub_twist.publish(out_twist)


    def storeData(self, event):
        actors_data = {}
        step_data = {}
        actors_data['ego_vehicle'] = {
                'type' = self.data[self.current_data_index].get('ego_vehicle').get('type'),
                'pose' = [self.current_pose.position.x, self.current_pose.position.y, self.current_pose.position.y, self.quatToYaw(self.current_pose.orientation)]
                'size' = self.data[self.current_data_index].get('ego_vehicle').get('size')
                }

        for id, actor in self.data[self.current_data_index].get('actors'):
            actors_data[id] = {
                'type' = actor.get('type'),
                'pose' = actor.get('pose')
                'size' = actor.get('ego_vehicle').get('size')
                }

        step_data = {
            'time' : rospy.Time.now(),
            'actors' : actors_data,
            'intervention' : self.intervened
            }
        self.time_step_data.append(step_data)


    def saveData(self):
        with open(self.out_data_file, 'wb') as f:
            pickle.dump(time_step_data, f)


if __name__ == '__main__':
    auto_intervention = AutoIntervention(sys.args[1], sys.args[2], sys.args[3])
