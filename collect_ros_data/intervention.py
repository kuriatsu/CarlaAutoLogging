#!/usr/bin/env python
# -*- coding:utf-8 -*-

import rospy
import time
import pickle
import sys
import numpy as np

from autoware_msgs.msg import VehicleCmd
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
        self.get_carla_speed = False
        self.max_intervention_count = 1
        self.intervention_count = 0
        self.last_intervention_time = None
        self.last_intervention_mileage = None
        self.start_intervention_time = 33.0
        self.allow_intervention = False
        self.intervention_gap = 1.0
        self.current_mileage = 0.0
        self.progress_as_succeed = 90.0
        self.last_obstacle_time = None
        self.obstacle_detection_gap = 0.5

        f = open("last_int_distance.txt", "r")
        self.start_intervention_mileage = float(f.read())
        f.close()

        self.pub_twist = rospy.Publisher('/vehicle_cmd', VehicleCmd, queue_size=1)
        self.pub_intervention = rospy.Publisher('/is_intervened', Bool, queue_size=1)
        self.pub_string = rospy.Publisher('/intervention_message', String, queue_size=1)
        self.sub_carla_speed = rospy.Subscriber('/vehicle_speed_carla', Float32, self.carlaSpeedCb)
        self.sub_detection_range = rospy.Subscriber('/detection_range', MarkerArray, self.detectionRangeCb)
        self.sub_twist = rospy.Subscriber('/twist_raw', TwistStamped, self.twistCb)
        self.sub_initial_pose = rospy.Subscriber('/initialpose', PoseWithCovarianceStamped, self.initialposeCb)
        self.sub_simulate_progress = rospy.Subscriber('/simulate_progress', Float32, self.simulateProgressCb)
        self.sub_mileage_progress = rospy.Subscriber('/mileage_progress', Float32, self.mileageProgressCb)


    def carlaSpeedCb(self, msg):
        self.carla_speed = msg.data


    def detectionRangeCb(self, msg):

        self.is_obstacle_on_path = False
        if self.last_obstacle_time is not None:
            if rospy.Time.now() - self.last_obstacle_time < rospy.Duration(self.obstacle_detection_gap):
                self.is_obstacle_on_path = True

        for marker in msg.markers:
            if marker.ns == 'Stop Line' and marker.color.g == 1.0: # for deceleration object
                self.is_obstacle_on_path = True
                self.last_obstacle_time = rospy.Time.now()

    def simulateProgressCb(self, msg):
        self.current_progress = msg.data

    def mileageProgressCb(self, msg):
        self.current_mileage = msg.data

    def twistCb(self, msg):

        if self.carla_speed is None:
            self.setZeroSpeed()
            return

        vel_eps = 0.3 # allow rate
        # angular_multiply_rate = 4.0

        carla_speed = self.carla_speed
        autoware_speed = msg.twist.linear.x

        out_twist = VehicleCmd()
        out_twist.twist_cmd.twist = msg.twist
        # out_twist.twist_cmd.twist.angular.z *= angular_multiply_rate
        if self.last_intervention_time is not None:
            if rospy.Time.now() - self.last_intervention_time > rospy.Duration(self.intervention_gap):
                self.intervention_count += 1
                self.last_intervention_time = None

        self.allow_intervention = (self.is_intervention_trial and self.current_progress > self.start_intervention_time and self.current_mileage > self.start_intervention_mileage and self.intervention_count < self.max_intervention_count)
        print("start_time: {}, start_mileage:{}, count{}".format(self.start_intervention_time, self.start_intervention_mileage, self.intervention_count))

        if self.allow_intervention:
            if autoware_speed > carla_speed * (1.0 + vel_eps) and self.is_obstacle_on_path:
                self.pub_string.publish(String(data='Brake'))
                out_twist.twist_cmd.twist.linear.x = carla_speed
                self.pub_intervention.publish(Bool(data=True))
                self.last_intervention_time = rospy.Time.now()
                self.last_intervention_mileage = self.current_mileage

            elif autoware_speed < carla_speed * (1.0 - vel_eps) and self.is_obstacle_on_path:
                self.pub_string.publish(String(data='Accel'))
                out_twist.twist_cmd.twist.linear.x = carla_speed
                self.pub_intervention.publish(Bool(data=True))
                self.last_intervention_time = rospy.Time.now()
                self.last_intervention_mileage = self.current_mileage

            else:
                self.pub_string.publish(String(data=''))
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
        if self.last_intervention_mileage is not None:
            print("write last intervention mileage")
            with open("last_int_distance.txt", "w") as f:
                f.write(str(self.last_intervention_mileage))

        elif self.current_progress > self.progress_as_succeed:
            print("no intervention and finished")
            with open("last_int_distance.txt", "w") as f:
                f.write(str(0.0))


if __name__ == '__main__':
    rospy.init_node('auto_intervention_node')
    auto_intervention = AutoIntervention(sys.argv[1])
    rospy.spin()
    auto_intervention.log_int_mileage()
