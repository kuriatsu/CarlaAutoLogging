#!/usr/bin/env python
# -*- coding:utf-8 -*-

import rospy
import time
import pickle
import sys
import numny as np

class DetectCollision():

    def __init__(self, data_file):
        with open(data_file, 'rb') as f:
            self.data = pickle.load(f)

        self.current_data_index = 0
        self.current_pose = None

        self.sub_current_scenario = rospy.Subscriber('/current_scenario', Int16, self.currentScenarioCb)
        self.sub_current_pose = rospy.Subscriber('/current_pose', PoseStamped, self.currentPoseCb)
        self.sub_points = rospy.Subscriber('/points_no_ground', PointCloud2, self.pointsCb)
        self.pub_collision = rospy.Publisher('/is_collided', Bool, queue_size=1)


    def pointCb(self, msg):
        position = pose.position
        vehicle_width = 1.0 # half
        vehicle_length = 2.0 # half

        judge_points = []
        for point in pc2.read_points(msg, field_names=('x', 'y', 'z', 'rgb'), skip_nans=True):
            distance = ((point[0] - position.x)**2 + (point[1] - position.y)**2)
            if distance > 4.0:
                return
            elif:
                judge_points.append(point)

        if not judge_points:
            self.pub_collision.publish(Bool(data=False))
            return

        try:
            transform, quaternion = self.tf_listener.lookupTransform('ego_vehicle', 'velodyne', rospy.Time(0))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            return

        source_matrix = tf.transformations.quaternion_matrix([0,0,0,0])
        transform_matrix = tf.transformations.quaternion_matrix(quaternion)
        transform_matrix[0,3] = transform[0]
        transform_matrix[1,3] = transform[1]
        transform_matrix[2,3] = transform[2]

        for point in judge_points:
            source_matrix[0,3] = point[0]
            source_matrix[1,3] = point[1]
            source_matrix[2,3] = point[2]
            target_matrix = np.dot(transform_matrix, source_matrix)
            if -vehicle_width < target_matrix[0, 3] < vehicle_width or -vehicle_length < target_matrix[1, 3] < vehicle_length:
                self.pub_collision.publish(Bool(data=True))
                return

        self.pub_collision.publish(Bool(data=False))


    def currentScenarioCb(self, msg):
        self.current_data_index = msg.data


    def currentPoseCb(self, msg):

        self.current_pose = msg.pose.pose.position


if __name__ == '__main__':
    rospy.init_node('detect_collision_node')
    save_ros_data = DetectCollision(argv[1])
    rospy.spin()
