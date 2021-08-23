#!/usr/bin/env python
# -*- coding:utf-8 -*-

import rospy
import time
import pickle
import sys
import numpy as np

from autoware_msgs.msg import VehicleCmd
from geometry_msgs.msg import PoseStamped, Point32, Twist, TwistStamped
from sensor_msgs.msg import PointCloud, ChannelFloat32
from std_msgs.msg import Int32, Bool, Float32, String
from visualization_msgs.msg import MarkerArray, Marker
import tf

class AutoIntervention():

    def __init__(self, intervention):
        self.intervention = bool(int(intervention))
        self.current_pose = None
        self.time_step_data = []
        self.avoid_deceleration = False
        self.carla_speed = None

        self.pub_twist = rospy.Publisher('/vehicle_cmd', VehicleCmd, queue_size=1)
        self.pub_intervention = rospy.Publisher('/is_intervened', Bool, queue_size=1)
        self.pub_string = rospy.Publisher('/intervention_message', String, queue_size=1)
        self.sub_carla_speed = rospy.Subscriber('/vehicle_speed_carla', Float32, self.carlaSpeedCb)
        self.sub_detection_range = rospy.Subscriber('/detection_range', MarkerArray, self.detectionRangeCb)
        self.sub_twist = rospy.Subscriber('/twist_raw', TwistStamped, self.twistCb)


    def carlaSpeedCb(self, msg):
        self.carla_speed = msg.data

    def detectionRangeCb(self, msg):

        self.avoid_deceleration = False
        for marker in msg.markers:
            if marker.ns == 'Stop Line' and marker.color.g == 1.0:
                self.avoid_deceleration = True


    def twistCb(self, msg):

        if self.carla_speed is None:
            return

        vel_range = 0.3 # allow rate
        curve_vel = 2.5 # m/s
        avoid_stop_vel = 4.0
        angular_multiply_rate = 4.0
        curve_angular = 0.07

        carla_speed = self.carla_speed
        autoware_speed = msg.twist.linear.x

        out_twist = VehicleCmd()
        out_twist.twist_cmd.twist = msg.twist
        out_twist.twist_cmd.twist.angular.z *= angular_multiply_rate

        if self.intervention:
            if autoware_speed > carla_speed * (1.0 + vel_range):
                print('\nbrake (over speed)')
                # out_twist.twist_cmd.twist.linear.x = carla_speed
                self.pub_intervention.publish(Bool(data=False))
                # self.pub_string.publish(String(data='Brake'))

            elif autoware_speed < carla_speed * (1.0 - vel_range) and self.avoid_deceleration:
                if abs(out_twist.twist_cmd.twist.angular.z) < curve_angular:
                    print('\nstraight obstacle accel')
                    self.pub_string.publish(String(data='Accel'))
                    out_twist.twist_cmd.twist.linear.x = carla_speed
                    self.pub_intervention.publish(Bool(data=True))

                else:
                    print('\ncurve obstacle accel')
                    out_twist.twist_cmd.twist.linear.x = carla_speed
                    self.pub_intervention.publish(Bool(data=True))
                    self.pub_string.publish(String(data='Accel'))

            # if autoware_speed == 0.0 and carla_speed == 0.0:
            #     print('avoid stop')
            #     out_twist.twist_cmd.twist.linear.x = avoid_stop_vel
            #     self.pub_intervention.publish(Bool(data=False))
            else:
                self.pub_string.publish(String(data=''))
                self.pub_intervention.publish(Bool(data=False))

        else:
            # if autoware_speed == 0.0:
            #     print('avoid stop')
            #     out_twist.twist_cmd.twist.linear.x = avoid_stop_vel
            #     self.pub_intervention.publish(Bool(data=False))

            self.pub_string.publish(String(data=''))
            self.pub_intervention.publish(Bool(data=False))

        sys.stdout.write('\r' + str(out_twist.twist_cmd.twist.angular.z) + ' : ' + str(autoware_speed) + ' : ' + str(carla_speed))
        self.pub_twist.publish(out_twist)


if __name__ == '__main__':
    rospy.init_node('auto_intervention_node')
    auto_intervention = AutoIntervention(sys.argv[1])
    rospy.spin()
