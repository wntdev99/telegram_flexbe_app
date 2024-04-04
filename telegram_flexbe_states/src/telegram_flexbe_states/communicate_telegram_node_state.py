#!/usr/bin/python3

import sys
import time
import rospy
import telebot
from flexbe_core import EventState, Logger
from std_srvs.srv import Trigger

class CommunicateTelegramNodeState(EventState):

    def __init__(self, message_service):
        super(CommunicateTelegramNodeState, self).__init__(outcomes=['behavior', 'wait'],
                                                          output_keys = ['get_message'],
                                                          input_keys = ['state'])
        self._message_service = message_service
        rospy.wait_for_service(self._message_service)

    def execute(self, userdata):
        if userdata.state:
            state_service = rospy.ServiceProxy(self._message_service, Trigger)
            reponse = state_service()
            if reponse.success:
                userdata.get_message = reponse.message
                return 'behavior'
        else:
            return 'wait'