#!/usr/bin/env python

from flexbe_core import EventState
from flexbe_core.proxy import ProxyPublisher
from geometry_msgs.msg import PoseWithCovarianceStamped

class InitializationKobuki(EventState):
    def __init__(self, topic):
        super(InitializationKobuki, self).__init__(outcomes=['OK'])
        self._topic = topic
        self._pub = ProxyPublisher({self._topic: PoseWithCovarianceStamped})

    def on_enter(self, userdata):
        self._pub.publish(self._topic, self.initialize_pose())

    def execute(self, userdata):
        return 'OK'
    
    def initialize_pose(self):
        initial_pose = PoseWithCovarianceStamped()
        initial_pose.header.frame_id = 'map'
        initial_pose.pose.pose.position.x = -0.18223335643389374
        initial_pose.pose.pose.position.y = 0.48117395235413735
        initial_pose.pose.pose.position.z = 0.0
        initial_pose.pose.pose.orientation.x = 0.0
        initial_pose.pose.pose.orientation.y = 0.0
        initial_pose.pose.pose.orientation.z = -0.1940315132327261
        initial_pose.pose.pose.orientation.w = 0.9809952965598858
        initial_pose.pose.covariance = [0.2016037788937209, 0.015400391742152048, 0.0, 0.0, 0.0, 0.0, 0.015400391742152048, 0.23355430723951517, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.06230573151326867]
        return initial_pose