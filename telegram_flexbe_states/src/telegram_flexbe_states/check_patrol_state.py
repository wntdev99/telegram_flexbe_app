#!/usr/bin/python3

import os
import rospy
from flexbe_core import EventState, Logger

CHECK_LIST = ['patrol_1', 'patrol_2', 'patrol_3', 'finish']

class CheckPotralState(EventState):
    def __init__(self, image_path):
        super(CheckPotralState, self).__init__(outcomes = ['navigation', 'finish'],
                                                   output_keys = ['pose_name'])
        self._check_arg = None
        self._image_path = image_path
        
    def execute(self, userdata):
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