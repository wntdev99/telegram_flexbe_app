#!/usr/bin/python3

import rospy
from flexbe_core import EventState, Logger
from ultralytics import YOLO
from PIL import Image
import cv2
import re
import time

class CheckPerson(EventState):
    def __init__(self, image_path, result_path, model_path):
        super(CheckPerson, self).__init__(outcomes = ['results', 'fail'],
                                          output_keys = ['results'],
                                          input_keys = ['pose_name'])
        self._image_path = image_path
        self._result_path = result_path
        self._model_path = model_path

    def execute(self, userdata):
        if self.check_image(userdata.pose_name):
            userdata.results = f"{self.check_person(userdata.pose_name)}명 있습니다."    
            return 'results'
        else:
            return 'fail'

    def _split_number(self, text):
        numbers =re.findall(r'\d+', text)
        if len(numbers):
            return numbers[0]
        else:
            return False

    def check_person(self, text):
        file_num = self._split_number(text)
        if int(file_num) == 1:
            file_num = ''
        try:
            with open(f"{self._result_path}/predict{file_num}/labels/image0.txt", "r") as file:
                file_content = file.readlines()
                return len(file_content)
        except:
            return 0
        
    def check_image(self, pose_name):
        try:
            model = YOLO(self._model_path)
            im1 = cv2.imread(f"{self._image_path}/{pose_name}.jpg")
            model.predict(source=im1, save=True, save_txt=True, retina_masks = True , classes=0)
            return True
        except Exception as e:
            Logger.loginfo(f"{e}")
            return False