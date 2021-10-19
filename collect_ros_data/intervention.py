#!/usr/bin/env python
# -*- coding:utf-8 -*-

import rospy
import time
import pickle
import sys
import numpy as np

from autoware_msgs.msg import VehicleCmd, DetectedObjectArray
from geometry_msgs.msg import PoseStamped, Point32, Twist, TwistStamped, PoseWithCovarianceStamped
from sensor_msgs.msg import PointCloud, ChannelFloat32
from std_msgs.msg import Int32, Bool, Float32, String
from visualization_msgs.msg import MarkerArray, Marker
import tf

class AutoIntervention():

    def __init__(self, intervention):
        self.is_intervention_trial = bool(int(intervention))
        self.current_pose = None
        self.time_step_data = []
        self.is_obstacle_on_path = False
        self.carla_speed = None

        self.progress_as_succeed = 90.0
        self.current_mileage = 0.0

        # remove detection noise
        # self.last_obstacle_time = None
        # self.obstacle_detection_gap = 1.0
        # self.last_intervention_mileage = None
        self.last_intervention_time = None
        self.last_intervention_twist = None
        self.intervention_duration = 0.5

        self.intervention_count = 0
        # self.allow_intervention = False

        # intervention control
        self.start_intervention_time = 33.0
        self.current_progress = 0.0
        self.max_intervention_count = 1
        self.objects = None
        self.intervened_target = []
        self.obstacle = None

        try:
            f = open("./intervened_target.pickle", "rb")
            buf = pickle.load(f)
            self.no_intervention_list = buf.get("last_list") + buf.get("no_list")
            f.close()
        except Exception as e:
            print(e)
            self.no_intervention_list = []

        self.pub_twist = rospy.Publisher('/vehicle_cmd', VehicleCmd, queue_size=1)
        self.pub_intervention = rospy.Publisher('/is_intervened', Bool, queue_size=1)
        self.pub_string = rospy.Publisher('/intervention_message', String, queue_size=1)
        self.sub_carla_speed = rospy.Subscriber('/vehicle_speed_carla', Float32, self.carlaSpeedCb)
        self.sub_detection_range = rospy.Subscriber('/detection_range', MarkerArray, self.detectionRangeCb)
        self.sub_twist = rospy.Subscriber('/twist_raw', TwistStamped, self.twistCb)
        self.sub_initial_pose = rospy.Subscriber('/initialpose', PoseWithCovarianceStamped, self.initialposeCb)
        self.sub_simulate_progress = rospy.Subscriber('/simulate_progress', Float32, self.simulateProgressCb)
        self.sub_mileage_progress = rospy.Subscriber('/mileage_progress', Float32, self.mileageProgressCb)
        self.sub_obstacle = rospy.Subscriber('/obstacle', Marker, self.obstacleCb)
        self.sub_objects = rospy.Subscriber('/detection/contour_tracker/objects', DetectedObjectArray, self.objectsCb)

    def carlaSpeedCb(self, msg):
        self.carla_speed = msg.data


    def detectionRangeCb(self, msg):

        self.is_obstacle_on_path = False
        # remove noise
        # if self.last_obstacle_time is not None:
        #     if rospy.Time.now() - self.last_obstacle_time < rospy.Duration(self.obstacle_detection_gap):
        #         self.is_obstacle_on_path = True

        for marker in msg.markers:
            if marker.ns == 'Stop Line' and marker.color.g == 1.0: # for deceleration object
                self.is_obstacle_on_path = True
                # self.last_obstacle_time = rospy.Time.now()

    def obstacleCb(self, msg):
        if self.objects is None:
            return

        self.obstacle = self.findClosestObject(msg.pose, self.objects)

    def findClosestObject(self, pose, objects):
        min_dist = 1000000
        closest_object = None
        for object in objects:
            dist = (pose.position.x - object.pose.position.x)**2 + (pose.position.y - object.pose.position.y)**2
            if min_dist > dist:
                min_dist = dist
                closest_object = object

        return closest_object.id

    def objectsCb(self, msg):
        self.objects = msg.objects

    def simulateProgressCb(self, msg):
        self.current_progress = msg.data

    def mileageProgressCb(self, msg):
        self.current_mileage = msg.data

    def twistCb(self, msg):

        if self.carla_speed is None:
            self.setZeroSpeed()
            return

        vel_eps = 0.2 # allow rate

        carla_speed = self.carla_speed
        autoware_speed = msg.twist.linear.x

        out_twist = VehicleCmd()
        out_twist.twist_cmd.twist = msg.twist

        if self.last_intervention_time is not None and rospy.Time.now() - self.last_intervention_time < rospy.Duration(self.intervention_duration):
            self.pub_string.publish(String(data='keep'))
            self.pub_intervention.publish(Bool(data=True))
            out_twist.twist_cmd.twist.linear.x = self.last_intervention_twist.twist_cmd.twist.linear.x

        else:
            is_allow_intervention = self.is_intervention_trial and \
                                      self.current_progress > self.start_intervention_time and \
                                      (len(self.intervened_target) < self.max_intervention_count or self.obstacle in self.intervened_target) and \
                                      self.obstacle not in self.no_intervention_list and \
                                      self.is_obstacle_on_path

            print("start_time: {}, obstacle:{}, list{}, no_list{}".format(self.start_intervention_time, self.obstacle, self.intervened_target, self.no_intervention_list))

            if is_allow_intervention:
                if autoware_speed > carla_speed * (1.0 + vel_eps):
                    self.pub_string.publish(String(data='Brake'))
                    out_twist.twist_cmd.twist.linear.x = carla_speed * 0.5
                    self.pub_intervention.publish(Bool(data=True))
                    self.last_intervention_time = rospy.Time.now()
                    self.last_intervention_twist = out_twist
                    if self.obstacle not in self.intervened_target:
                        self.intervened_target.append(self.obstacle)

                elif autoware_speed < carla_speed * (1.0 - vel_eps):
                    self.pub_string.publish(String(data='Accel'))
                    out_twist.twist_cmd.twist.linear.x = carla_speed * 1.5
                    self.pub_intervention.publish(Bool(data=True))
                    self.last_intervention_time = rospy.Time.now()
                    self.last_intervention_twist = out_twist
                    if self.obstacle not in self.intervened_target:
                        self.intervened_target.append(self.obstacle)

                else:
                    self.pub_string.publish(String(data='No Need'))
                    self.pub_intervention.publish(Bool(data=False))

            else:
                self.pub_string.publish(String(data=''))
                self.pub_intervention.publish(Bool(data=False))

        # sys.stdout.write('\r' + str(out_twist.twist_cmd.twist.angular.z) + ' : ' + str(autoware_speed) + ' : ' + str(carla_speed))
        self.pub_twist.publish(out_twist)


    def initialposeCb(self, msg):
        self.carla_speed = None
        self.setZeroSpeed()


    def setZeroSpeed(self):
        out_twist = VehicleCmd()
        out_twist.header.stamp = rospy.Time.now()
        out_twist.twist_cmd.twist.linear.x = 0.0
        out_twist.twist_cmd.twist.angular.z = 0.0
        self.pub_twist.publish(out_twist)

    def log_int_mileage(self):

        out_dict = {"no_list" : self.no_intervention_list, "last_list" : self.intervened_target}
        with open("intervened_target.pickle", "wb") as f:
            pickle.dump(out_dict, f)


if __name__ == '__main__':
    rospy.init_node('auto_intervention_node')
    auto_intervention = AutoIntervention(sys.argv[1])
    rospy.spin()
    auto_intervention.log_int_mileage()
