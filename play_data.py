#!/usr/bin/env python
# -*- coding:utf-8 -*-

import rospy
import time
import pickle
import sys
import numpy as np
import subprocess

from std_msgs.msg import Header
from autoware_msgs.msg import Waypoint, LaneArray, Lane, DetectedObjectArray, DetectedObject
from geometry_msgs.msg import PoseStamped, Point32, Quaternion, PointStamped, PoseWithCovarianceStamped, PolygonStamped, Polygon, Pose
from sensor_msgs.msg import PointCloud2, PointField
import sensor_msgs.point_cloud2 as pc2
import tf
# import pcl


class PlayCarlaData():

    def __init__(self, data_file):

        with open(data_file, 'rb') as f:
            self.data = pickle.load(f)

        self.tf_broadcaster = tf.TransformBroadcaster()
        self.tf_listener = tf.TransformListener()
        self.current_data_index = 0

        self.pub_cloud = rospy.Publisher('/points_no_ground', PointCloud2, queue_size=5)
        self.pub_object = rospy.Publisher('/detection/contour_tracker/objects', DetectedObjectArray, queue_size=5)
        self.pub_initial_pose = rospy.Publisher('/initialpose', PoseWithCovarianceStamped, queue_size=5)
        self.pub_waypoint = rospy.Publisher('/lane_waypoints_array', LaneArray, queue_size=1, latch=True)
        self.sub_current_pose = rospy.Subscriber('/current_pose', PoseStamped, self.currentPoseCb)

        self.init_pose(self.data[0])
        self.setWaypoint(self.data)
        self.pubActorTf(self.data[0].get('actors'))


    def yawToQuat(self, yaw):
        quat = tf.transformations.quaternion_from_euler(0.0, 0.0, np.radians(yaw))
        return Quaternion(x=quat[0], y=quat[1], z=quat[2], w=quat[3])


    def pubActorTf(self, actors):
        for id, actor in actors.items():
            quaternion = self.yawToQuat(actor.get('waypoint')[3])
            self.tf_broadcaster.sendTransform(
                (actor.get('waypoint')[0], actor.get('waypoint')[1], actor.get('waypoint')[2]),
                (quaternion.x, quaternion.y, quaternion.z, quaternion.w),
                rospy.Time.now(),
                str(id),
                'map'
                )

    def pubActorCloud(self, actors):
        cloud_header = Header(stamp=rospy.Time.now(), frame_id='velodyne')
        cloud_field = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name='rgb', offset=12, datatype=PointField.UINT32, count=1),
            ]
        point_cloud = []

        for id, actor in actors.items():
            polygon = self.calcPolygon(str(id), actor, cloud_header.frame_id)
            for polygon_point in polygon.points:
                point = [polygon_point.x, polygon_point.y, polygon_point.z, 0xffffff]
                point_cloud.append(point)

        cloud = pc2.create_cloud(cloud_header, cloud_field, point_cloud)
        self.pub_cloud.publish(cloud)


    def calcPolygon(self, source_frame, actor, target_frame):
        polygon_stamped = Polygon()
        buf_header = Header(stamp=rospy.Time(0), frame_id=source_frame)

        horizontal_edge_num = int(actor.get('size')[0] * 2 / 0.1)
        for i in range(0, horizontal_edge_num):
            buf_point_stamped = PointStamped()
            buf_point_stamped.header = buf_header
            buf_point_stamped.point.x = -actor.get('size')[0] + 0.1 * i
            buf_point_stamped.point.y = actor.get('size')[1]
            buf_point_stamped.point.z = 0.0
            transformed_point = self.tf_listener.transformPoint(target_frame, buf_point_stamped)
            polygon_stamped.points.append(transformed_point.point)

            buf_point_stamped.point.y *= -1
            transformed_point = self.tf_listener.transformPoint(target_frame, buf_point_stamped)
            polygon_stamped.points.append(transformed_point.point)

        vertical_edge_num = int(actor.get('size')[1] * 2 / 0.1)
        for i in range(0, vertical_edge_num):
            buf_point_stamped = PointStamped()
            buf_point_stamped.header = buf_header
            buf_point_stamped.point.x = actor.get('size')[0]
            buf_point_stamped.point.y = -actor.get('size')[1] + 0.1 * i
            buf_point_stamped.point.z = 0.0
            transformed_point = self.tf_listener.transformPoint(target_frame, buf_point_stamped)
            polygon_stamped.points.append(transformed_point.point)

            buf_point_stamped.point.x *= -1
            transformed_point = self.tf_listener.transformPoint(target_frame, buf_point_stamped)
            polygon_stamped.points.append(transformed_point.point)

        return polygon_stamped


    def pubActorObject(self, autors):
        object_array = DetectedObjectArray()
        for id, actor in actors.items():
            object = DetectedObject()
            object.header = Header(stamp=rospy.Time.now(), frame_id='map')
            object.id = id
            object.score = 80
            object.valid = false
            object.space_frame = ''
            object.pose.position.x = actor.get('waypoint')[0]
            object.pose.position.y = actor.get('waypoint')[1]
            object.pose.position.z = actor.get('waypoint')[2]
            object.pose.orientation = self.yawToQuat(actor.get('waypoint')[3])
            object.dimention.x = actor.get('size')[0]
            object.dimention.y = actor.get('size')[1]
            object.dimention.z = actor.get('size')[2]
            object.velocity = actor.get('waypoint')[4]
            object.convex_hull.polygon = calcPolygon(str(id), actor, object.header.frame_id)
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
            if self.current_data_index > len(self.data):
                exit()

        self.pubActorTf(self.data[self.current_data_index].get('actors'))
        self.pubActorCloud(self.data[self.current_data_index].get('actors'))


    def setWaypoint(self, data_list):
        lane_array = LaneArray()
        lane = Lane()
        lane.header.stamp = rospy.Time.now()
        lane.header.frame_id = 'map'
        lane.lane_id = 1
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

        lane_array.lanes.append(lane)
        self.pub_waypoint.publish(lane_array)


    def init_pose(self, data):

        waypoint = data.get('waypoint')
        quat = self.yawToQuat(waypoint[3])
        pose = waypoint[0:3] + [quat.x, quat.y, quat.z, quat.w]
        command = ['bash', 'set_initialpose.sh'] + [str(i) for i in pose]
        subprocess.call(command)
        print(command)


if __name__ == '__main__':
    rospy.init_node('carla_play_data_node')
    play_carla_data = PlayCarlaData(sys.argv[1])
    rospy.spin()
