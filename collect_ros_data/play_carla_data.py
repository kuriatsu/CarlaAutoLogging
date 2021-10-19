#!/usr/bin/env python
# -*- coding:utf-8 -*-

import rospy
import time
import pickle
import sys
import numpy as np
import subprocess

from std_msgs.msg import Header, Int32, Float32, Float32MultiArray, String
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
            buf = pickle.load(f)

        self.data = buf.get('data')
        self.waypoint = buf.get('waypoint')
        self.tf_broadcaster = tf.TransformBroadcaster()
        self.tf_listener = tf.TransformListener()
        self.config_replanner = None
        self.closest_waypoint = 0
        self.current_data_index = 0
        self.current_pose = None
        self.start_time = None

        self.pub_cloud = rospy.Publisher('/points_no_ground', PointCloud2, queue_size=5)
        self.pub_object = rospy.Publisher('/detection/contour_tracker/objects', DetectedObjectArray, queue_size=5)
        self.pub_initial_pose = rospy.Publisher('/initialpose', PoseWithCovarianceStamped, queue_size=5)
        self.pub_waypoint = rospy.Publisher('/based/lane_waypoints_raw', LaneArray, queue_size=1, latch=True)
        self.pub_config_replanner = rospy.Publisher('/config/waypoint_replanner', ConfigWaypointReplanner, queue_size=1)
        self.pub_carla_speed = rospy.Publisher('/vehicle_speed_carla', Float32, queue_size=1)
        self.pub_ego_vehicle_size = rospy.Publisher('/ego_vehicle/size', Float32MultiArray, queue_size=1, latch=True)
        self.pub_ego_vehicle_type = rospy.Publisher('/ego_vehicle/type', String, queue_size=1, latch=True)
        self.pub_simulate_progress = rospy.Publisher('/simulate_progress', Float32, queue_size=1)
        self.pub_mileage_progress = rospy.Publisher('/mileage_progress', Float32, queue_size=1)
        self.pub_mileage = rospy.Publisher('/mileage', Float32, queue_size=1)

        # self.pub_twist = rospy.Publisher('/vehicle_cmd', VehicleCmd, queue_size=1)
        self.sub_config_replanner = rospy.Subscriber('/config/waypoint_replanner', ConfigWaypointReplanner, self.configReplannerCb)
        self.sub_current_pose = rospy.Subscriber('/current_pose', PoseStamped, self.currentPoseCb)

        self.setFirstData(0)
        self.start_time = rospy.get_time()
        self.timer = rospy.Timer(rospy.Duration(0.5), self.timerCb)


    def setFirstData(self, index):

        self.setWaypoint(self.waypoint)

        ego_vehicle_info = self.data[index].get('actors').get('ego_vehicle')
        self.pub_ego_vehicle_size.publish(Float32MultiArray(data=ego_vehicle_info.get('size')))
        self.pub_ego_vehicle_type.publish(String(data=ego_vehicle_info.get('type')))

        actor_data = self.data[index].get('actors')
        result = False
        while not result:
            self.pubActorTf(actor_data)
            result = self.pubActorCloud(actor_data)

        self.pubActorObject(actor_data)

        self.initialPose(self.data[index])


    def initialPose(self, data):
        waypoint = data.get('actors').get('ego_vehicle').get('pose')[0:3]
        quat = self.yawToQuat(data.get('actors').get('ego_vehicle').get('pose')[3])
        pose = waypoint[0:3] + [quat.x, quat.y, quat.z, quat.w]
        command = ['bash', 'set_initialpose.sh'] + [str(i) for i in pose]
        subprocess.call(command)


    def setWaypoint(self, waypoint):
        lane_array = LaneArray()
        lane = Lane()
        lane.header.stamp = rospy.Time.now()
        lane.header.frame_id = 'map'
        lane.lane_id = 1
        for data in waypoint:
            waypoint = Waypoint()
            waypoint.pose.pose.position.x = data.get('x')
            waypoint.pose.pose.position.y = data.get('y')
            waypoint.pose.pose.position.z = data.get('z')
            waypoint.pose.pose.orientation = self.yawToQuat(data.get('yaw'))
            waypoint.gid = 1
            waypoint.wpstate.event_state = 1
            waypoint.lane_id = 1
            lane.waypoints.append(waypoint)

        lane_array.lanes.append(lane)
        self.pub_waypoint.publish(lane_array)


    # def setZeroSpeed(self):
    #     out_twist = VehicleCmd()
    #     out_twist.header.stamp = rospy.Time.now()
    #     out_twist.twist_cmd.twist.linear.x = 0.0
    #     out_twist.twist_cmd.twist.angular.z = 0.0
    #     self.pub_twist.publish(out_twist)


    def timerCb(self, event):
        if self.current_pose is None:
            return

        elapsed_time = rospy.get_time() - self.start_time
        next_scenario_time = self.data[self.current_data_index + 1].get('time') - self.data[0].get('time')
        if elapsed_time > next_scenario_time:
            self.current_data_index += 1
        self.pub_simulate_progress.publish(Float32(data=(100 * self.current_data_index/(len(self.data)-1))))

        actor_data = self.data[self.current_data_index].get('actors')
        self.pubActorTf(actor_data)
        self.pubActorCloud(actor_data)
        self.pubActorObject(actor_data)

        current_waypoint = self.getClosestWaypoint(self.waypoint, self.current_pose.position)
        # print(self.waypoint, current_waypoint, self.current_pose)
        self.pubConfigReplanner(self.waypoint[current_waypoint].get('speed_limit'))
        self.pub_carla_speed.publish(Float32(data=self.waypoint[current_waypoint].get('speed')))
        self.pub_mileage_progress.publish(Float32(data=(100*current_waypoint/(len(self.waypoint)-1))))
        self.pub_mileage.publish(Float32(data=(current_waypoint)))

        # Judge finish
        if self.current_data_index == len(self.data)-1:
        # if self.current_data_index == len(self.data)-1 or 100*current_waypoint/(len(self.waypoint)-1) >= 95.0:
            rospy.signal_shutdown("finish")


    def pubConfigReplanner(self, speed_limit):
        if self.config_replanner is not None and self.config_replanner.velocity_max == speed_limit:
            return

        config = ConfigWaypointReplanner()
        config.replanning_mode=True
        config.realtime_tuning_mode=True
        config.resample_mode=True
        config.resample_interval=1
        config.replan_curve_mode=True
        config.overwrite_vmax_mode=True
        config.replan_endpoint_mode=False
        config.velocity_max = speed_limit
        config.velocity_min = 20.0
        config.radius_thresh= 50
        config.radius_min= 10.0
        config.accel_limit= 0.5
        config.decel_limit= 0.5
        config.velocity_offset= 0
        config.braking_distance= 5
        config.end_point_offset= 0
        config.use_decision_maker= False
        self.config_replanner = config
        self.pub_config_replanner.publish(config)


    def pubActorTf(self, actors):
        for id, actor in actors.items():
            if id == 'ego_vehicle':
                continue
            quaternion = self.yawToQuat(actor.get('pose')[3])
            self.tf_broadcaster.sendTransform(
                (actor.get('pose')[0], actor.get('pose')[1], actor.get('pose')[2]),
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
            if id == 'ego_vehicle':
                continue
            polygon = self.calcPolygon(str(id), actor, cloud_header.frame_id)
            if polygon is None:
                continue

            for polygon_point in polygon.points:
                point = [polygon_point.x, polygon_point.y, polygon_point.z, 0xffffff]
                point_cloud.append(point)

        if not point_cloud:
            print('failed to get polygon')
            return False

        cloud = pc2.create_cloud(cloud_header, cloud_field, point_cloud)
        self.pub_cloud.publish(cloud)
        return True


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
            source_matrix[0,3] = -actor.get('size')[0] + 0.2 * i
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
            target_point = Point()
            target_point.x = target_matrix[0, 3]
            target_point.y = target_matrix[1, 3]
            target_point.z = target_matrix[2, 3]
            polygon_stamped.points.append(target_point)

        vertical_edge_num = int(actor.get('size')[1] * 2 / 0.2)
        for i in range(0, vertical_edge_num):
            source_matrix[0,3] = actor.get('size')[0]
            source_matrix[1,3] = -actor.get('size')[1] + 0.2 * i
            source_matrix[2,3] = 0.0
            target_matrix = np.dot(transform_matrix, source_matrix)
            target_point = Point()
            target_point.x = target_matrix[0, 3]
            target_point.y = target_matrix[1, 3]
            target_point.z = target_matrix[2, 3]
            polygon_stamped.points.append(target_point)

            source_matrix[0,3] *= -1
            target_matrix = np.dot(transform_matrix, source_matrix)
            target_point = Point()
            target_point.x = target_matrix[0, 3]
            target_point.y = target_matrix[1, 3]
            target_point.z = target_matrix[2, 3]
            polygon_stamped.points.append(target_point)

        return polygon_stamped


    def pubActorObject(self, actors):
        object_array = DetectedObjectArray()
        for id, actor in actors.items():
            if id == 'ego_vehicle':
                continue
            object = DetectedObject()
            object.header = Header(stamp=rospy.Time.now(), frame_id='map')
            object.id = id
            object.label = actor.get('type')
            object.score = 80
            object.valid = False
            object.space_frame = ''
            object.pose.position.x = actor.get('pose')[0]
            object.pose.position.y = actor.get('pose')[1]
            object.pose.position.z = actor.get('pose')[2]
            object.pose.orientation = self.yawToQuat(actor.get('pose')[3])
            object.dimensions.x = actor.get('size')[0]
            object.dimensions.y = actor.get('size')[1]
            object.dimensions.z = actor.get('size')[2]
            object.velocity.linear.x = actor.get('speed')
            # object.convex_hull.polygon = calcPolygon(str(id), actor, object.header.frame_id)
            # object.convex_hull.header = object.header
            object.pose_reliable = True
            object.velocity_reliable = True
            object.acceleration_reliable = False
            object_array.objects.append(object)

        self.pub_object.publish(object_array)

    # not used (filtered when data collection)
    def delOverlapObjects(self, actor_list, waypoint):
        for id, actor in actor_list.items():
            if actor.get('type').startswith('walker'):
                continue

            if self.isOverlapOnEgoTrajectry(actor, waypoint):
                print(actor.get('type'))
                del actor_list[id]

    # not used (filtered when data collection)
    def isOverlapOnEgoTrajectry(self, actor, waypoint):
        for data in waypoint:
            dist = ((actor.get('pose')[0] - data.get('x'))**2 + (actor.get('pose')[1] - data.get('y'))**2)**0.5
            if dist > 1.8:
                continue

            angle = actor.get('pose')[3] - data.get('yaw')
            # print(dist, angle, actor.get('size')[0])
            if abs(angle) > 45 and (360.0 - abs(angle)) > 45:
                continue

            return True

        return False


    def getClosestWaypoint(self, waypoint, point):
        min_dist = 1000000
        closest_waypoint = 0
        for i, data in enumerate(waypoint):
            dist = (point.x - data.get('x')) ** 2 + (point.y - data.get('y')) ** 2
            if min_dist > dist:
                min_dist = dist
                closest_waypoint = i

        return closest_waypoint


    def configReplannerCb(self, msg):
        self.config_replanner = msg


    def yawToQuat(self, yaw):
        quat = tf.transformations.quaternion_from_euler(0.0, 0.0, np.radians(yaw))
        return Quaternion(x=quat[0], y=quat[1], z=quat[2], w=quat[3])


    def currentPoseCb(self, msg):
        self.current_pose = msg.pose


if __name__ == '__main__':
    rospy.init_node('carla_play_data_node')
    play_carla_data = PlayCarlaData(sys.argv[1])
    rospy.spin()
