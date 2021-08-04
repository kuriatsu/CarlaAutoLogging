#!/usr/bin/env python
# -*- coding:utf-8 -*-

import rospy
import time
import pickle
import sys
import numny as np
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Int16, Bool


class SaveRosData():
    def __init__(self, load_data_file, out_data_file):
        with open(load_data_file, 'rb') as f:
            self.data = pickle.load(f)

        self.current_data_index = 0
        self.current_pose = None
        self.is_intervened = False

        self.out_data_file = out_data_file
        self.sub_current_scenario = rospy.Subscriber('/current_scenario', Int16, self.currentScenarioCb)
        self.sub_current_pose = rospy.Subscriber('/current_pose', PoseStamped, self.currentPoseCb)
        self.sub_intervention = rospy.Subscriber('/is_intervened', Bool, self.interventionCb)
        rospy.Timer(rospy.Duration(0.5), self.storeData)


    def currentScenarioCb(self, msg):
        self.current_data_index = msg.data


    def currentPoseCb(self, msg):
        self.current_pose = msg.pose


    def interventionCb(self, msg):
        self.is_intervened = msg.data


    def storeData(self, event):
        actors_data = {}
        step_data = {}
        actors_data['ego_vehicle'] = {
                'type' = self.data[self.current_data_index].get('ego_vehicle').get('type'),
                'pose' = [self.current_pose.position.x, self.current_pose.position.y, self.current_pose.position.y, self.quatToYaw(self.current_pose.orientation)]
                'size' = self.data[self.current_data_index].get('ego_vehicle').get('size')
                }

        for id, actor in self.data[self.current_data_index].get('actors').items():
            actors_data[id] = actor

        step_data = {
            'time' : rospy.Time.now(),
            'actors' : actors_data,
            'intervention' : self.is_intervened
            }
        self.time_step_data.append(step_data)


    def saveData(self):
        with open(self.out_data_file, 'wb') as f:
            pickle.dump(time_step_data, f)


if __name__ == '__main__':
    try:
        rospy.init_node('save_ros_data_node')
        save_ros_data = SaveRosData(argv[1], argv[2])
        rospy.spin()
    finally:
        del save_ros_data.saveData()
