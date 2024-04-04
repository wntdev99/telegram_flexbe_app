#!/usr/bin/python3

import pandas as pd
from flexbe_core import EventState, Logger
from geometry_msgs.msg import PoseStamped

class GetGoalPose(EventState):
    def __init__(self, excel_path):
        super(GetGoalPose, self).__init__(outcomes = ['navigation'], 
                                          input_keys = ['pose_name'],
                                          output_keys = ['goal_pose'])
        self._excel_path = excel_path
        self._pose_data = []
        self._goal_pose = PoseStamped()
        
    def on_enter(self, userdata):
        self.load_excel_data(userdata.pose_name)

        self._goal_pose.header.frame_id = self._pose_data[0][1]
        self._goal_pose.pose.position.x = self._pose_data[0][2]
        self._goal_pose.pose.position.y = self._pose_data[0][3]
        self._goal_pose.pose.position.z = self._pose_data[0][4]
        self._goal_pose.pose.orientation.x = self._pose_data[0][5]
        self._goal_pose.pose.orientation.y = self._pose_data[0][6]
        self._goal_pose.pose.orientation.z = self._pose_data[0][7]
        self._goal_pose.pose.orientation.w = self._pose_data[0][8]

    def execute(self, userdata):
        userdata.goal_pose = self._goal_pose
        return 'navigation'

    def load_excel_data(self, pose_name):
        try:
            df = pd.read_excel(self._excel_path)
            specific_row_data = df.loc[df['pose_name'] == int(pose_name)]
            self._pose_data = specific_row_data.values.tolist()
        except:
            df = pd.read_excel(self._excel_path)
            specific_row_data = df.loc[df['pose_name'] == pose_name]
            self._pose_data = specific_row_data.values.tolist()