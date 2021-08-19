#!/usr/bin/env python
# -*- coding:utf-8 -*-

import rospy
import numpy as np
import tf

from geometry_msgs.msg import TwistStamped, PoseStamped, Point, PointStamped
from autoware_msgs.msg import Lane, VehicleCmd
from std_msgs.msg import Header



class PurePursuit():

    def __init__(self):

        self.look_ahead_distance = 3.0
        self.waypoint = None
        self.current_twist = None
        self.last_waypoint = 0
        # self.current_pose = None

        self.sub_waypoint = rospy.Subscriber('/final_waypoints', Lane, self.waypointCb)
        self.sub_current_pose = rospy.Subscriber('/current_pose', PoseStamped, self.currentPoseCb)
        self.sub_current_twist = rospy.Subscriber('/current_velocity', TwistStamped, self.currentTwistCb)
        self.pub_twist = rospy.Publisher('/vehicle_cmd_pure_pursuit', VehicleCmd, queue_size=1)
        self.pub_point = rospy.Publisher('/debug', PointStamped, queue_size=1)


    def waypointCb(self, msg):
        self.waypoint = msg.waypoints
        # print(len(msg.waypoints))


    def currentTwistCb(self, msg):
        self.current_twist = msg.twist


    def currentPoseCb(self, msg):

        if self.current_twist is None or self.waypoint is None:
            print('no current twist or waypoint')
            return

        closest_waypoint_idx = self.getClosestWaypoint(self.waypoint, msg.pose.position)
        target_waypoint_idx = self.getTargetWaypoint(self.waypoint, closest_waypoint_idx)
        print(closest_waypoint_idx, target_waypoint_idx)
        if target_waypoint_idx is None:
            print('no target waypoint')
            return
        target_waypoint = self.waypoint[target_waypoint_idx]
        speed, omega = self.culcTwist(msg.pose, target_waypoint.pose.pose, self.current_twist.linear.x, target_waypoint.twist.twist.linear.x, self.look_ahead_distance)

        out_cmd = VehicleCmd()
        out_cmd.twist_cmd.twist.linear.x = speed
        out_cmd.twist_cmd.twist.angular.z = omega
        self.pub_twist.publish(out_cmd)


    def culcTwist(self, current_pose, target_pose, current_speed, target_speed, look_ahead_distance):
        alpha = np.arctan((target_pose.position.y - current_pose.position.y) / (target_pose.position.x - current_pose.position.x)) - self.quatToYaw(current_pose.orientation)
        omega = (2 * target_speed * np.sin(alpha)) / look_ahead_distance

        return target_speed, omega


    def quatToYaw(self, quat):

        euler = tf.transformations.euler_from_quaternion((quat.x, quat.y, quat.z, quat.w))
        return euler[2]



    def getClosestWaypoint(self, waypoint, point):

        min_dist = 1000000
        closest_waypoint = self.last_waypoint
        for i in range(0, len(waypoint)):
            data = waypoint[i].pose.pose.position
            dist = (point.x - data.x) ** 2 + (point.y - data.y) ** 2
            print(dist**0.5)
            if min_dist > dist:
                min_dist = dist
                closest_waypoint = i

        self.last_waypoint = closest_waypoint
        self.pub_point.publish(PointStamped(header=Header(frame_id='map'), point=waypoint[closest_waypoint].pose.pose.position))
        return closest_waypoint


    def getTargetWaypoint(self, waypoint, closest_waypoint_idx):
        if closest_waypoint_idx is None:
            return

        target_waypoint = None
        buf_dist = 0.0
        last_position = waypoint[closest_waypoint_idx].pose.pose.position

        for i in range(closest_waypoint_idx+1, len(waypoint)):
            buf_data = waypoint[i].pose.pose.position
            buf_dist += ((last_position.x - buf_data.x)**2 + (last_position.y - buf_data.y)**2)**0.5
            last_position = buf_data
            target_waypoint = i
            if buf_dist >= self.look_ahead_distance:
                break


        return target_waypoint


if __name__ == '__main__':
    rospy.init_node('pure_pursuit_node')
    pure_pursuit = PurePursuit()
    rospy.spin()
