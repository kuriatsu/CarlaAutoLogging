#!/usr/bin/env python
# -*- coding:utf-8 -*-

import rospy
import time
import pickle
import sys
import numpy as np

from autoware_msgs.msg import Waypoint, LaneArray, Lane
from geometry_msgs.msg import PoseStamped, Point32, Quaternion
from sensor_msgs.msg import PointCloud2, PointField, ChannelFloat32, convertPointCloudToPointCloud
import tf
import pcl



class PlayCarlaData():

    def __init__(self, data_file):
        self.pub_objects = rospy.Publisher('/points_raw', PointCloud, queue_size=5)
        self.pub_initial_pose = rospy.Publisher('/initial_pose', PoseStamped, queue_size=1)
        self.pub_waypoint = rospy.Publisher('/lane_waypoints_array', LaneArray, queue_size=1, latch=True)
        self.sub_current_pose = rospy.Subscriber('/current_pose', PoseStamped, self.currentPoseCb)
        self.tf_broadcaster = tf.TransformBroadcaster()
        self.tf_listener = tf.TransformListener()

        with open(data_file, 'rb') as f:
            self.data = pickle.load(f)

        self.current_data_index = 0
        self.init_pose(self.data[0])
        self.pubActorTf(self.data[0].get('actors'))


    def yawToQuat(self, yaw):
        quat = tf.transformations.quaternion_from_euler(0, 0, yaw)
        return Quaternion(x=quat[0], y=quat[1], z=quat[2], w=quat[3])


    def pubActorTf(self, actors):

        for id, actor in actors.items():

            quaternion = self.yawToQuat(actor.get('waypoint')[4])

            self.tf_broadcaster.sendTransform(
                (actor.get('waypoint')[0], actor.get('waypoint')[1], actor.get('waypoint')[2]),
                (quaternion.x, quaternion.y, quaternion.z, quaternion.w),
                rospy.Time.now(),
                str(id),
                'map')


    def pubActorObjInfo(self, actors):
        point_cloud_header = Header()
        point_cloud_header.stamp = rospy.Time.now()
        point_cloud_header.frame_id = 'map'
        points = []
        points_field = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name='rgb', offset=12, datatype=PointField.UINT32, count=1),
        ]


        for id, actor in actors.items():
            horizontal_edge_num = int(actor.get('size').x * 2 / 0.1)
            vertical_edge_num = int(actor.get('size').y * 2 / 0.1)
            local_point_cloud = PointCloud()
            global_point_cloud = PointCloud()


            for i in range(0, horizontal_edge_num):
                point = [
                    -actor.get('size').x + 0.1 * i,
                    actor.get('size').y,
                    0.0,
                    0xffffff
                    ]
                points.append(point)
                point = [
                    -actor.get('size').x + 0.1 * i,
                    -actor.get('size').y,
                    0.0,
                    0xffffff
                    ]
                points.append(point)

            for i in range(0, vertical_edge_num):
                point = [
                    actor.get('size').x,
                    -actor.get('size').y + 0.1 * i,
                    0.0,
                    0xffffff
                    ]
                points.append(point)
                point = [
                    -actor.get('size').x,
                    -actor.get('size').y + 0.1 * i,
                    0.0,
                    0xffffff
                    ]
                points.append(point)

            self.tf_listener.transformPointCloud(id, point_cloud_header.stamp, local_point_cloud, 'map', global_point_cloud)
            point_cloud.points.append(global_point_cloud.points)

        self.pub_objects.publish(point_cloud)


    def poseTransform(source_pose, source_frame, target_frame, tf_listener):
        """transform pose from source_frame to target_frame
        Args:
            source_pose: (geometry_msg/Pose)
            source_frame: ex. map
            target_frame: ex. world
            tf_listener: instance generated as global or class member like #Line 9 (do not define every time when you use this function, it takes a lot of time)
        Return:
            target_pose: transformed pose (geometry_msg/Pose)
        """
        source_matrix = tf.transformations.quaternion_matrix([source_pose.orientation.x, source_pose.orientation.y, source_pose.orientation.z, source_pose.orientation.w])
        source_matrix[0,3] = source_pose.position.x
        source_matrix[1,3] = source_pose.position.y
        source_matrix[2,3] = source_pose.position.z

        try:
            transform, quaternion = tf_listener.lookupTransform(target_frame, source_frame, rospy.Time(0))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            return

        transform_matrix = tf.transformations.quaternion_matrix(quaternion)
        transform_matrix[0,3] = transform[0]
        transform_matrix[1,3] = transform[1]
        transform_matrix[2,3] = transform[2]

        target_matrix = np.dot(transform_matrix, source_matrix)
        target_quaternion = tf.transformations.quaternion_from_matrix(target_matrix)

        target_pose = Pose()
        target_pose.position = Point(target_matrix[0,3], target_matrix[1,3], target_matrix[2,3])
        target_pose.orientation = Quaternion(target_quaternion[0], target_quaternion[1], target_quaternion[2], target_quaternion[3])

        return target_pose


    def currentPoseCb(self, msg):
        current_wp = self.data[self.current_data_index].get('waypoint')
        next_wp = self.data[self.current_data_index+1].get('waypoint')
        dist_to_current_wp = (msg.pose.position.x - current_wp[0]) ** 2 + (msg.pose.position.y - current_wp[1]) ** 2
        dist_to_next_wp = (msg.pose.position.x - next_wp[0]) ** 2 + (msg.pose.position.y - next_wp[1]) ** 2

        if dist_to_current_wp > dist_to_next_wp:
            self.current_data_index += 1

        self.pubActorTf(self.data[self.current_data_index].get('actors'))
        self.pubActorObjInfo(self.data[self.current_data_index].get('actors'))


    def pubWaypoint(self, data_list):
        lane_array = LaneArray()
        lane = Lane()

        for data in data_list:
            waypoint = Waypoint()
            waypoint.pose.pose.position.x = data.get('waypoint')[0]
            waypoint.pose.pose.position.y = data.get('waypoint')[1]
            waypoint.pose.pose.position.z = data.get('waypoint')[2]
            waypoint.pose.pose.orientation = self.yawToQuat(data.get('waypoint')[3])
            waypoint.twist.twist.linear.x = data.get('waypoint')[4]
            waypoint.gid = 1
            waypoint.wpstate.event_state = 1
            waypoint.lane_id = 1
            lane.waypoints.append(waypoint)

        lane_array.header.stamp = rospy.Time.now()
        lane_array.header.frame_id = 'map'
        lane_array.lane_id = 1
        lane_array.lanes.append(lane)
        self.pub_waypoint.publish(lane_array)


    def init_pose(self, data):
        pose = PoseStamped()
        pose.pose.position.x = data.get('waypoint')[0]
        pose.pose.position.y = data.get('waypoint')[1]
        pose.pose.position.z = data.get('waypoint')[2]
        pose.pose.orientation = self.yawToQuat(data.get('waypoint')[3])
        self.pub_initial_pose.publish(pose)


if __name__ == '__main__':
    rospy.init_node('carla_play_data_node')

    play_carla_data = PlayCarlaData(sys.argv[1])

    rospy.spin()
