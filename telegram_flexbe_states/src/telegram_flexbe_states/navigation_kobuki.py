#!/usr/bin/python3

import time
from flexbe_core import EventState, Logger
from flexbe_core.proxy import ProxyActionClient
from mbf_msgs.msg import MoveBaseAction, MoveBaseGoal


class NavigationKobuki(EventState):
    def __init__(self, move_base_action):

        super(NavigationKobuki, self).__init__(outcomes = ['results'],
                                               input_keys = ['goal_pose', 'pose_name'],
                                               output_keys = ['results'])
        self._result = ''
        self._error = False
        self._goal = MoveBaseGoal()
        self._topic = move_base_action
        self._client = ProxyActionClient({self._topic: MoveBaseAction})

    def on_enter(self, userdata):
        self._goal.target_pose = userdata.goal_pose
        self._error = False
        try:
            self._client.send_goal(self._topic, self._goal)
        except Exception as e:
            Logger.logwarn('Failed to send the MoveBaseAction command:\n%s' % str(e))
            self._error = True
        
    def execute(self, userdata):
        if self._client.has_result(self._topic):
            self._result = self._client.get_result(self._topic)
            userdata.results = f"현재 꼬부기가 {userdata.pose_name}에 도착하였습니다 !"
            return 'results'
        
    def on_exit(self, userdata):
        if self._client.is_available(self._topic) and not self._client.has_result(self._topic):
            self._client.cancel(self._topic)
            Logger.loginfo('Cancelled active action goal.')