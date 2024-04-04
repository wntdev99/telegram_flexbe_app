#!/usr/bin/python3

import rospy
from flexbe_core import EventState, Logger
from std_srvs.srv import Trigger

class ChangeRobotState(EventState):

    def __init__(self, service):
        super(ChangeRobotState, self).__init__(outcomes = ['OK'],
                                                output_keys = ['state'])
        self._service = service
        self._check_state = None
        rospy.wait_for_service(self._service)
        
    def on_enter(self, userdata):
        state_service = rospy.ServiceProxy(self._service, Trigger)
        reponse = state_service()
        self._check_state = reponse.success
        Logger.loginfo(f"Currently robot state is {self._check_state}.")

    def execute(self, userdata):
        userdata.state = self._check_state
        return 'OK'
        