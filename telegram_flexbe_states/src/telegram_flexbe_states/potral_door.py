#!/usr/bin/python3

import pcl
import time
import math
import rospy
import rostopic
import numpy as np
import sensor_msgs.point_cloud2 as pc2
from flexbe_core import EventState, Logger
from flexbe_core.proxy import ProxySubscriberCached

class PointCloudTestState(EventState):
    def __init__(self, pointcloud2_topic, message_door_open, message_door_close, blocking=True, clear=False):

        super(PointCloudTestState, self).__init__(outcomes = ['complete'],
                                           output_keys = ['results'],
                                           input_keys = ['pose_name'])
        self._sub_topic = pointcloud2_topic
        self._blocking = blocking
        self._clear = clear
        self._connected = False
        self._point_data = None
        self._message_door_open = message_door_open
        self._message_door_close = message_door_close   
        self._standard_angle = 4.0
        self._num_iterations = 1
        self._door_angle = []
        
        if not self._connect():
            Logger.logwarn('Topic %s for state %s not yet available.\n'
                           'Will try again when entering the state...' % (self._sub_topic, self.name))

    def execute(self, userdata):
        if self._connect():
            if self._sub.has_msg(self._sub_topic) or not self._blocking:
                self._point_data = self._sub.get_last_msg(self._sub_topic)

                pcl_data = self.ros_to_pcl(self._point_data)

                # Split range
                pcl_data = self.do_passthrough(pcl_data, 'y', -2, 0)

                # Min value
                min_z_data = np.apply_along_axis(lambda a: np.min(a), 0, np.array(pcl_data))[2]

                # Get plane normal vector
                door_pcl_data = self.do_passthrough(pcl_data, 'z', min_z_data + 0.01, min_z_data + 0.10)
                door_coefficients = self.do_ransac_plane_normal_segmentation(door_pcl_data, 0.05)
                wall_pcl_data = self.do_passthrough(pcl_data, 'z', min_z_data + 0.20, math.inf)
                wall_coefficients = self.do_ransac_plane_normal_segmentation(wall_pcl_data, 0.05)

                # calculation_normal_vector
                wall_door_angle = self.angle_between_vectors(door_coefficients, wall_coefficients)
                rospy.loginfo(f"wall_door_angle: {wall_door_angle}")

                wall_door_angle = self.angle_between_vectors(door_coefficients, wall_coefficients)
                self._door_angle.append(wall_door_angle)
                Logger.loginfo(f"{wall_door_angle}")
                if len(self._door_angle) == self._num_iterations:
                    if self.check_angle(sum(self._door_angle) / len(self._door_angle)):
                        userdata.results = f"{userdata.pose_name} {self._message_door_open}"
                    else:
                        userdata.results = f"{userdata.pose_name} {self._message_door_close}"
                    return 'complete'
                
        else:
            Logger.logwarn('Topic %s still not available, giving up.' % self._sub_topic)


    def on_exit(self, userdata):
        self._door_angle = []
        if self._connected and self._clear and self._sub.has_msg(self._sub_topic):
            self._sub.remove_last_msg(self._sub_topic)


    def _connect(self):
        global_topic = self._sub_topic
        if global_topic[0] != '/':
            global_topic = rospy.get_namespace() + global_topic
        msg_type, msg_topic, _ = rostopic.get_topic_class(global_topic)
        if msg_topic == global_topic:
            self._sub = ProxySubscriberCached({self._sub_topic: msg_type})
            self._connected = True
            return True
        return False

    def check_angle(self, angle):
        if angle > self._standard_angle:
            return True
        else:
            return False

    def ros_to_pcl(self, ros_cloud):
        points_list = []

        for data in pc2.read_points(ros_cloud, skip_nans=True):
            points_list.append([data[0], data[1], data[2], data[3]])

        pcl_data = pcl.PointCloud_PointXYZRGB()
        pcl_data.from_list(points_list)

        return pcl_data
    
    def do_passthrough(self, pcl_data, filter_axis, axis_min, axis_max):
        passthrough = pcl_data.make_passthrough_filter()
        passthrough.set_filter_field_name(filter_axis)
        passthrough.set_filter_limits(axis_min, axis_max)
        return passthrough.filter()
    
    def do_ransac_plane_normal_segmentation(self, point_cloud, input_max_distance):
        segmenter = point_cloud.make_segmenter_normals(ksearch=500)
        segmenter.set_optimize_coefficients(True)
        segmenter.set_model_type(pcl.SACMODEL_NORMAL_PLANE) 
        segmenter.set_normal_distance_weight(0.1)
        segmenter.set_method_type(pcl.SAC_RANSAC)
        segmenter.set_max_iterations(500)
        segmenter.set_distance_threshold(input_max_distance)
        indices, coefficients = segmenter.segment()
        return coefficients[:3]

    def angle_between_vectors(self, door_coefficients, wall_coefficients):
        cos_theta = np.dot(door_coefficients,wall_coefficients)
        angle = math.acos(cos_theta) * 180.0 / math.pi
        return angle