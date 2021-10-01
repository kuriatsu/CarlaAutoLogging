#!/usr/bin/env python
# -*- coding:utf-8 -*-

import rospy
import time
import pickle
import sys
import tf
import numpy as np

from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import Float32, String, Header, ColorRGBA
from geometry_msgs.msg import Point, Vector3, Quaternion


class PlayRosData():
    def __init__(self, data_file):
        with open(data_file, 'rb') as f:
            data = pickle.load(f)
        self.data = data.get("drive_data")
        self.waypoint = data.get("waypoint")

        self.pub_waypoint = rospy.Publisher('/waypoint_marker', Marker, queue_size=1, latch=True)
        self.pub_object = rospy.Publisher('/objects', MarkerArray, queue_size = 1)
        self.pub_simulate_progress = rospy.Publisher('/simulate_progress', Float32, queue_size=1)
        self.pub_mileage_progress = rospy.Publisher('/mileage_progress', Float32, queue_size=1)
        self.pub_collision = rospy.Publisher('/collision_message', String, queue_size = 1)
        self.pub_intervention = rospy.Publisher('/intervention_message', String, queue_size = 1)
        self.pub_intervention_target = rospy.Publisher('/obstacle', Marker, queue_size = 1)
        self.pub_time = rospy.Publisher('elapsed_time', String, queue_size=1)

        self.makeWaypoint(self.waypoint)
        self.current_data_index = 0
        self.timer = rospy.Timer(rospy.Duration(0.5), self.timerCb)


    def timerCb(self, event):
        marker_list = MarkerArray()

        if len(self.data) <= 1:
            print("no data contains")
            rospy.signal_shutdown("no drive data contains")

        try:
            for id, actor in self.data[self.current_data_index].get('actors').items():
                marker = Marker(header=Header(stamp=rospy.Time.now(), frame_id='world'))
                marker.type = Marker.CUBE
                marker.action = Marker.ADD
                marker.lifetime = rospy.Duration(0.5)
                marker.ns = str(id)
                marker.pose.position = self.listToPoint(actor.get('pose'))
                marker.pose.orientation = self.yawToQuat(actor.get('pose')[3])
                marker.scale = self.sizeToVector(actor.get('size'))
                if id == 'ego_vehicle':
                    marker.color.r = 1.0
                    marker.color.g = 0.0
                    marker.color.b = 0.0
                    marker.color.a = 1.0
                else:
                    marker.color.r = 0.0
                    marker.color.g = 1.0
                    marker.color.b = 0.0
                    marker.color.a = 1.0

                # marker.frame_locked = True
                marker_list.markers.append(marker)

            self.pub_object.publish(marker_list)
            self.pub_simulate_progress.publish(self.data[self.current_data_index].get('simulate_progress'))
            self.pub_mileage_progress.publish(self.data[self.current_data_index].get('mileage_progress'))
            self.pub_time.publish(str(self.data[self.current_data_index].get('time') - self.data[0].get('time')))

            if self.data[self.current_data_index].get('collision'):
                self.pub_collision.publish(String(data='collide'))
            else:
                self.pub_collision.publish(String(data=''))

            if self.data[self.current_data_index].get('intervention'):
                self.pub_intervention.publish(String(data='accel'))
                marker = Marker(header=Header(stamp=rospy.Time.now(), frame_id='world'))
                marker.type = Marker.CUBE
                marker.action = Marker.ADD
                marker.pose.position = self.listToPoint(self.data[self.current_data_index].get('intervention_target'))
                marker.pose.orientation = self.yawToQuat(0.0)
                marker.scale = self.sizeToVector([1.0, 1.0, 1.0])
                marker.color.r = 1.0
                marker.color.g = 1.0
                marker.color.b = 0.0
                marker.color.a = 1.0
                marker.lifetime = rospy.Duration(0.5)
                marker.frame_locked = False
                self.pub_intervention_target.publish(marker)
            else:
                self.pub_intervention.publish(String(data=''))

            self.current_data_index += 1
            if self.current_data_index == len(self.data)-1:
                rospy.signal_shutdown("finish")

        except Exception as e:
            print(e)
            return


    def listToPoint(self, list):
        point = Point()
        point.x = list[0]
        point.y = list[1]
        point.z = list[2]

        return point


    def sizeToVector(self, list):
        vector = Vector3()
        vector.x = list[0]*2
        vector.y = list[1]*2
        vector.z = list[2]*2

        return vector

    def yawToQuat(self, yaw):
        quat = tf.transformations.quaternion_from_euler(0.0, 0.0, yaw)
        return Quaternion(x=quat[0], y=quat[1], z=quat[2], w=quat[3])


    def makeWaypoint(self, waypoint):
        marker = Marker(header=Header(stamp=rospy.Time.now(), frame_id='world'))
        marker.type = Marker.SPHERE_LIST
        marker.action = Marker.ADD
        marker.id = 0
        marker.lifetime = rospy.Duration(0)
        marker.ns = "waypoint"
        marker.scale = Vector3(x=1.0, y=1.0, z=1.0)
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 1.0
        marker.color.a = 1.0
        for data in waypoint:
            point = Point(x=data.get("x"), y=data.get("y"), z=data.get("z"))
            marker.points.append(point)

        self.pub_waypoint.publish(marker)


    def removeMarker(self):
        marker = Marker(header=Header(stamp=rospy.Time.now(), frame_id='world'))
        marker.action = Marker.DELETEALL
        self.pub_waypoint.publish(marker)


if __name__ == '__main__':
    rospy.init_node("play_ros_data_node")
    play_ros_data = PlayRosData(sys.argv[1])
    rospy.spin()
    play_ros_data.removeMarker()
