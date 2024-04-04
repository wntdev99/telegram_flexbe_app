#!/usr/bin/python3

import os
from flexbe_core import EventState, Logger

class ClearFileState(EventState):
    def __init__(self, image_path, result_path):
        super(ClearFileState, self).__init__(outcomes = ['next'])
        self._image_path = image_path
        self._result_path = result_path

    def execute(self, userdata):
        self.delete_folder(self._image_path)
        self.delete_folder(self._result_path)
        self.create_folder(self._image_path)
        return 'next'

    def on_exit(self, userdata):
        Logger.loginfo("Kobuki complete a work you commanded")

    def delete_folder(folder_path):
        if os.path.exists(folder_path):
            try:
                os.rmdir(folder_path)
            except OSError as e:
                Logger.loginfo(f"Error: {e}")

    def create_folder(folder_path):
        try:
            os.makedirs(folder_path)
        except OSError as e:
            Logger.loginfo(f"Error: {e}")