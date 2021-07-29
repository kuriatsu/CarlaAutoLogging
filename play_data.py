#!/usr/bin/env python
# -*- coding:utf-8 -*-

import rospy
import time
import pickle
import sys
import numpy as np

from autoware_msgs.msg import Waypoint, LaneArray, Lane, DetectedObjectArray, DetectedObject
from geometry_msgs.msg import PoseStamped, Point32, Quaternion, PointStamped, PoseWithCovarianceStamped, PolygonStamped, Polygon
from sensor_msgs.msg import PointCloud2, PointField
import sensor_msgs.point_cloud2 as pc2
import tf
import pcl


class PlayCarlaData():

    def __init__(self, data_file):
        self.pub_cloud = rospy.Publisher('/points_raw', PointCloud2, queue_size=1000)
        self.pub_cloud = rospy.Publisher('/detection/contour_tracker/objects', DetectedObjectArray, queue_size=5)
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
                'map'
                )

    def pubActorCloud(self, actors):
        cloud_header = Header(stamp=rospy.Time.now(), frame_id='map')
        cloud_field = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name='rgb', offset=12, datatype=PointField.UINT32, count=1),
            ]
        point_cloud = []

        for id, actor in actors.items():
            polygon = self.culcPolygon(id, actor)
            for polygon_point in polygon.points:
                point = [polygon_point.x, polygon_point.y, polygon_point.z, 0xffffff]
                point_cloud.append(point)

        cloud = pc2.create_cloud(cloud_header, cloud_field, point_cloud)
        self.pub_cloud.publish(cloud)


    def calcPolygon(self, frame_id, actor):
        polygon_stamped = Polygon()
        buf_header = Header(stamp=point_cloud_header, frame_id=frame_id)

        horizontal_edge_num = int(actor.get('size').x * 2 / 0.1)
        for i in range(0, horizontal_edge_num):
            buf_point_stamped = PointStamped()
            buf_point_stamped.header = buf_header
            buf_point_stamped.point.x = -actor.get('size').x + 0.1 * i
            buf_point_stamped.point.y = actor.get('size').y
            buf_point_stamped.point.z = 0.0
            transformed_point = tf.transformPoint('map', buf_point_stamped)
            polygon.points.append(transformed_point)

            buf_point_stamped.point.y *= -1
            transformed_point = tf.transformPoint('map', buf_point_stamped)
            polygon.points.append(transformed_point)

        vertical_edge_num = int(actor.get('size').y * 2 / 0.1)
        for i in range(0, vertical_edge_num):
            buf_point_stamped = PointStamped()
            buf_point_stamped.header = buf_header
            buf_point_stamped.point.x = actor.get('size').x
            buf_point_stamped.point.y = -actor.get('size').y + 0.1 * i
            buf_point_stamped.point.z = 0.0
            buf_point_stamped = Point()
            transformed_point = tf.transformPoint('map', buf_point_stamped)
            polygon.points.append(transformed_point)

            buf_point_stamped.point.x *= -1
            transformed_point = tf.transformPoint('map', buf_point_stamped)
            polygon.points.append(transformed_point)

        return polygon


    def pubActorObject(self, autors):
        object_array = DetectedObjectArray()
        for id, actor in actors.items():
            object = DetectedObject()
            object.header = Header(stamp=point_cloud_header, frame_id='map')
            object.id = id
            object.score = 80
            object.valid = false
            object.space_frame = ''
            object.pose.position.x = actor.get('waypoint')[0]
            object.pose.position.y = actor.get('waypoint')[1]
            object.pose.position.z = actor.get('waypoint')[2]
            object.pose.orientation = self.yawToQuat(actor.get('waypoint')[3])
            object.dimention.x = actor.get('size').x
            object.dimention.y = actor.get('size').y
            object.dimention.z = actor.get('size').z
            object.velocity = actor.get('waypoint')[4]
            object.convex_hull.polygon = calcPolygon(id, actor)
            object.convex_hull.header = object.header
            object.pose_reliable = True
            object.velocity_reliable = True
            object.acceleration_reliable = False
            object_array.append(object)

        self.pub_object.publish(object_array)


    def currentPoseCb(self, msg):
        current_wp = self.data[self.current_data_index].get('waypoint')
        next_wp = self.data[self.current_data_index+1].get('waypoint')
        dist_to_current_wp = (msg.pose.position.x - current_wp[0]) ** 2 + (msg.pose.position.y - current_wp[1]) ** 2
        dist_to_next_wp = (msg.pose.position.x - next_wp[0]) ** 2 + (msg.pose.position.y - next_wp[1]) ** 2

        if dist_to_current_wp > dist_to_next_wp:
            self.current_data_index += 1

        self.pubActorTf(self.data[self.current_data_index].get('actors'))
        self.pubActorCloud(self.data[self.current_data_index].get('actors'))


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
        pose_stamped = PoseStamped()
        pose_stamped.header = Header(stamp=rospy.Time.now(), frame_id='map')
        pose_stamped.pose.position.x = data.get('waypoint')[0]
        pose_stamped.pose.position.y = data.get('waypoint')[1]
        pose_stamped.pose.position.z = data.get('waypoint')[2]
        pose_stamped.pose.orientation = self.yawToQuat(data.get('waypoint')[3])
        pose_stamped = tf.transformPose('world', pose_stamped)
        out_pose = PoseWithCovarianceStamped()
        out_pose.header = pose_stamped.header()
        out_pose.pose = pose_stamped.pose
        out.pose.covariance[0] = 0.25
        out.pose.covariance[6*1 + 1] = 0.25
        out.pose.covariance[6*5 + 5] = 0.06853892326654787
        self.pub_initial_pose.publish(pose)


if __name__ == '__main__':
    rospy.init_node('carla_play_data_node')
    play_carla_data = PlayCarlaData(sys.argv[1])
    rospy.spin()
