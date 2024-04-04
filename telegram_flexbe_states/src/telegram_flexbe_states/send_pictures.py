#!/usr/bin/python3

import re
import telebot
from flexbe_core import EventState, Logger

class SendPictures(EventState):
    def __init__(self, token, chat_id, image_path, result_path):
        super(SendPictures, self).__init__(outcomes = ['next'],
                                          input_keys = ['pose_name'])
        self._token = token
        self._chat_id = chat_id
        self._image_path = image_path
        self._result_path = result_path
        self._bot = telebot.TeleBot(self._token)
        
    def execute(self, userdata):
        try:
            if userdata.pose_name.count('patrol'):
                with open(f"{self._image_path}{userdata.pose_name}.jpg", "rb") as image_file:
                    self._bot.send_photo(self.chat_id, image_file)
            else:
                num = int(self.split_number(userdata.pose_name))
                if num == 1:
                    num = ''
                Logger.loginfo(f"{self._result_path}/predict{num}/image0.jpg")
                with open(f"{self._result_path}/predict{num}/image0.jpg", "rb") as image_file:
                    self._bot.send_photo(self.chat_id, image_file)
        except Exception as e:
            Logger.logerr(f"Error message: {e}")

        self._bot.send_message(chat_id=self._chat_id , text="ㅤㅤㅤㅤㅤㅤㅤㅤㅤㅤ")
        return 'next'

    def split_number(self, text):
        numbers =re.findall(r'\d+', text)
        if len(numbers):
            return numbers[0]
        else:
            return False
        
    def send_results_message(self, message_text):
        self._bot = telebot.TeleBot(self._token)
        self._bot.send_message(chat_id=self._chat_id, text=message_text)