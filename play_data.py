#!/usr/bin/env python
# -*- coding:utf-8 -*-

import rospy
import time
import pickle
import sys
import numny as np

#from autoware_msgs.msg import DetectedObjectArray
from geometry_msgs.msg import PoseStamped, Point32
from sensor_msgs.msg import PointCloud, ChannelFloat32
import tf

class PlayCarlaData():

    def __init__(self, data_file):
        self.pub_objects = rospy.Publisher('/detection/contour_tracker/objects', DetectedObjectArray, queue_size=5)
        self.pub_initial_pose = rospy.Publisher('/initial_pose', PoseStamped, queue_size=1)
        self.sub_current_pose = rospy.Subscriber('/current_pose', PoseStamped, self.currentPoseCb)
        self.tf_broadcaster = tf.TransformBroadcaster()
        self.tf_listener = tf.TransformListener()

        with open(data_file, 'rb') as f:
            self.data = pickle.load(f)

        self.current_data_index = 0
        self.init_pose(data[0])


    def yawToQuat(yaw):
        quat = tf.transformation.quaternion_from_euler(0, 0, yaw)
        return Quaternion(x=quat[0], y=quat[1], z=quat[2], w=quat[3])


    def pubActorTf(self, actors):

        for id, actor in actors.items():

            quaternion = self.yawToQuat(actor.get('pose')[4])

            self.tf_broadcaster.sendTransform(
                (actor.get('pose')[0], actor.get('pose')[1], actor.get('pose')[2]),
                (quaternion.x, quaternion.y, quaternion.z, quaternion.w),
                rospy.Time.now(),
                id,
                'map')


    def pubActorObjInfo(self, actors):
        point_cloud = PointCloud()
        point_cloud.header.frame_id = 'map'
        point_cloud.header.stamp = rospy.Time.now()

        for id, actor in actors.items():
            horizontal_edge_num = int(actor.get('size').x * 2 / 0.1)
            vertical_edge_num = int(actor.get('size').y * 2 / 0.1)
            local_point_cloud = PointCloud()
            global_point_cloud = PointCloud()


            for i in range(0, horizontal_edge_num):
                point = Point32()
                channel = ChannelFloat32()
                point.x = -actor.get('size').x + 0.1 * i
                point.y = actor.get('size').y
                point.z = 0.0
                channel.name = 'intensity'
                channel.values.append(1.0)
                local_point_cloud.points.append(point)

                point.y = -actor.get('size').y
                local_point_cloud.points.append(point)

            for i in range(0, vertical_edge_num):
                point = Point32()
                channel = ChannelFloat32()
                point.x = actor.get('size').x
                point.y = -actor.get('size').y + 0.1 * i
                point.z = 0.0
                channel.name = 'intensity'
                channel.values.append(1.0)
                local_point_cloud.points.append(point)

                point.x = -actor.get('size').x
                local_point_cloud.points.append(point)

            self.tf_listener.transformPointCloud(id, point_cloud.header.stamp, local_point_cloud, 'map', global_point_cloud)
            point_cloud.points.append(global_point_cloud.points)

        self.pub_objects.publish(point_cloud)


    def currentPoseCb(self, msg):
        current_wp = self.data[self.current_data_index].get('waypoint')
        next_wp = self.data[self.current_data_index+1].get('waypoint')
        dist_to_current_wp = (msg.pose.position.x - current_wp[0]) ** 2 + (msg.pose.position.y - current_wp[1]) ** 2
        dist_to_next_wp = (msg.pose.position.x - next_wp[0]) ** 2 + (msg.pose.position.y - next_wp[1]) ** 2

        if dist_to_current_wp > dist_to_next_wp:
            self.current_data_index += 1

        self.pubActorTf(self.data[self.current_data_index].get('actors'))
        self.pubActorObjInfo(self.data[self.current_data_index].get('actors'))


    def init_pose(self, data)
        pose = PoseStamped()
        pose.position.x = data.get('waypoint')[0]
        pose.position.y = data.get('waypoint')[1]
        pose.position.z = data.get('waypoint')[2]
        pose.orientation = yawToQuat(data.get('waypoint')[3])
        self.pub_initial_pose(pose)


if __name__ == '__main__':
    rospy.init('carla_play_data_node')

    play_carla_data = PlayCarlaData(sys.argsp[1])

    rospy.spin()
