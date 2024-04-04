#!/usr/bin/python3

import os
import rospy
from flexbe_core import EventState, Logger

CHECK_LIST = ['check_1', 'check_2', 'check_3', 'check_4', 'check_5', 'check_6', 'finish']

class CheckNavigationState(EventState):
    def __init__(self, image_path):
        super(CheckNavigationState, self).__init__(outcomes = ['navigation', 'finish'],
                                                   output_keys = ['pose_name'])
        self._check_arg = ''
        self._image_path = image_path
        
    def execute(self, userdata):
        check_arg = ''
        for check_arg in CHECK_LIST:
            file_path = f"{self._image_path}{check_arg}.jpg"
            if os.path.exists(file_path):
                pass
            else:
                self._check_arg = check_arg
                break
        if check_arg == 'finish':
            return check_arg
        else:
            userdata.pose_name = self._check_arg
            return 'navigation'