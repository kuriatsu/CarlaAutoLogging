#!/usr/bin/env python
# -*- coding:utf-8 -*-

import rospy
import time
import pickle
import sys
import numpy as np
import subprocess

from std_msgs.msg import Header, Int32
from autoware_msgs.msg import Waypoint, LaneArray, Lane, DetectedObjectArray, DetectedObject
from autoware_config_msgs.msg import ConfigWaypointReplanner
from geometry_msgs.msg import PoseStamped, Point, Quaternion, PointStamped, PoseWithCovarianceStamped, PolygonStamped, Polygon, Pose
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
        self.current_pose = None
        self.config_replanner = None
        self.closest_waypoint = 0

        self.pub_cloud = rospy.Publisher('/points_no_ground', PointCloud2, queue_size=5)
        self.pub_object = rospy.Publisher('/detection/contour_tracker/objects', DetectedObjectArray, queue_size=5)
        self.pub_initial_pose = rospy.Publisher('/initialpose', PoseWithCovarianceStamped, queue_size=5)
        self.pub_waypoint = rospy.Publisher('/based/lane_waypoints_raw', LaneArray, queue_size=1, latch=True)
        self.pub_config_replanner = rospy.Publisher('/config/waypoint_replanner', ConfigWaypointReplanner, queue_size=1)
        self.pub_scenario = rospy.Publisher('/current_scenario', Int32, queue_size=1)
        self.sub_config_replanner = rospy.Subscriber('/config/waypoint_replanner', ConfigWaypointReplanner, self.configReplannerCb)
        self.sub_current_pose = rospy.Subscriber('/current_pose', PoseStamped, self.currentPoseCb)

        self.init_pose(self.data[0])
        time.sleep(1.0)
        self.setWaypoint(self.data)
        self.pubActorTf(self.data[0].get('actors'))
        self.pubConfigReplanner(self.data[0].get('speed_limit'))
        self.timer = rospy.Timer(rospy.Duration(0.1), self.timerCb)


    def init_pose(self, data):

        waypoint = data.get('waypoint')
        quat = self.yawToQuat(waypoint[3])
        pose = waypoint[0:3] + [quat.x, quat.y, quat.z, quat.w]
        command = ['bash', 'set_initialpose.sh'] + [str(i) for i in pose]
        subprocess.call(command)


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
            waypoint.gid = 1
            waypoint.wpstate.event_state = 1
            waypoint.lane_id = 1
            lane.waypoints.append(waypoint)

        lane_array.lanes.append(lane)
        self.pub_waypoint.publish(lane_array)


    def timerCb(self, event):
        if self.current_pose is None:
            return

        min_dist = 1000000
        closest_data_index = None
        for i, data in enumerate(self.data):
            waypoint = data.get('waypoint')
            position = self.current_pose.position
            dist = (position.x - waypoint[0]) ** 2 + (position.y - waypoint[1]) ** 2
            if min_dist > dist:
                min_dist = dist
                closest_data_index = i

        if closest_data_index > len(self.data) - 3:
            rospy.signal_shutdown("finish")

        self.pub_scenario.publish(Int32(data=closest_data_index))
        self.pubConfigReplanner(self.data[closest_data_index].get('speed_limit'))
        self.pubActorTf(self.data[closest_data_index].get('actors'))
        self.pubActorCloud(self.data[closest_data_index].get('actors'))


    def pubConfigReplanner(self, max_speed):
        if self.config_replanner is not None and self.config_replanner.velocity_max == max_speed:
            return

        config = ConfigWaypointReplanner()
        config.replanning_mode=True
        config.realtime_tuning_mode=True
        config.resample_mode=True
        config.resample_interval=1
        config.replan_curve_mode=True
        config.overwrite_vmax_mode=True
        config.replan_endpoint_mode=False
        config.velocity_max = max_speed
        config.velocity_min = 8.0
        config.radius_thresh= 50
        config.radius_min= 10.0
        config.accel_limit= 0.3
        config.decel_limit= 0.3
        config.velocity_offset= 0
        config.braking_distance= 5
        config.end_point_offset= 0
        config.use_decision_maker= False
        self.config_replanner = config
        self.pub_config_replanner.publish(config)


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
            if polygon is None:
                print('failed to get polygon')
                continue

            for polygon_point in polygon.points:
                point = [polygon_point.x, polygon_point.y, polygon_point.z, 0xffffff]
                point_cloud.append(point)

        cloud = pc2.create_cloud(cloud_header, cloud_field, point_cloud)
        self.pub_cloud.publish(cloud)


    def calcPolygon(self, source_frame, actor, target_frame):
        polygon_stamped = Polygon()
        buf_header = Header(stamp=rospy.Time(0), frame_id=source_frame)

        try:
            transform, quaternion = self.tf_listener.lookupTransform(target_frame, source_frame, rospy.Time(0))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            return

        source_matrix = tf.transformations.quaternion_matrix([0,0,0,0])
        transform_matrix = tf.transformations.quaternion_matrix(quaternion)
        transform_matrix[0,3] = transform[0]
        transform_matrix[1,3] = transform[1]
        transform_matrix[2,3] = transform[2]

        horizontal_edge_num = int(actor.get('size')[0] * 2 / 0.2)
        for i in range(0, horizontal_edge_num):
            source_matrix[0,3] = -actor.get('size')[0] + 0.1 * i
            source_matrix[1,3] = actor.get('size')[1]
            source_matrix[2,3] = 0.0
            target_matrix = np.dot(transform_matrix, source_matrix)
            target_point = Point()
            target_point.x = target_matrix[0, 3]
            target_point.y = target_matrix[1, 3]
            target_point.z = target_matrix[2, 3]
            polygon_stamped.points.append(target_point)

            source_matrix[1,3] *= -1
            target_matrix = np.dot(transform_matrix, source_matrix)
            target_point.x = target_matrix[0, 3]
            target_point.y = target_matrix[1, 3]
            target_point.z = target_matrix[2, 3]
            polygon_stamped.points.append(target_point)

        vertical_edge_num = int(actor.get('size')[1] * 2 / 0.2)
        for i in range(0, vertical_edge_num):
            target_point = Point()
            source_matrix[0,3] = actor.get('size')[0]
            source_matrix[1,3] = -actor.get('size')[1] + 0.1 * i
            source_matrix[2,3] = 0.0
            target_matrix = np.dot(transform_matrix, source_matrix)
            target_point.x = target_matrix[0, 3]
            target_point.y = target_matrix[1, 3]
            target_point.z = target_matrix[2, 3]
            polygon_stamped.points.append(target_point)

            source_matrix[0,3] *= -1
            target_matrix = np.dot(transform_matrix, source_matrix)
            target_point.x = target_matrix[0, 3]
            target_point.y = target_matrix[1, 3]
            target_point.z = target_matrix[2, 3]
            polygon_stamped.points.append(target_point)

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
        self.current_pose = msg.pose


    def configReplannerCb(self, msg):
        self.config_replanner = msg


    def yawToQuat(self, yaw):
        quat = tf.transformations.quaternion_from_euler(0.0, 0.0, np.radians(yaw))
        return Quaternion(x=quat[0], y=quat[1], z=quat[2], w=quat[3])




if __name__ == '__main__':
    rospy.init_node('carla_play_data_node')
    play_carla_data = PlayCarlaData(sys.argv[1])
    rospy.spin()
