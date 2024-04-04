#!/usr/bin/python3

import re
import telebot
import pandas as pd
from flexbe_core import EventState, Logger

SEND_MESSAGE = ['명령어 양식에 맞춰서 입력해주시기 바랍니다.', '꼬부기 출발합니다.', '꼬부기 충전시키겠습니다.', '순찰하고 오겠습니다.']

CALL_COMMAND = '호출'
DELIVERY_COMMAND = '배송'
PATROL_COMMAND = '순찰'
CHARGE_COMMAND = '충전'
FAIL_COMMAND = '실패'

COMMAND_DICT = {'호출':'call', '충전':'charge', '순찰':'patrol', '실패':'fail'}

class SeparateCommandMessageState(EventState):
    def __init__(self, token, chat_id, excel_path):
        super(SeparateCommandMessageState, self).__init__(outcomes = ['call', 'patrol', 'charge', 'fail'],
                                          output_keys = ['pose_name'],
                                          input_keys = ['get_message'])
        self._token = token
        self._message = None
        self._chat_id = chat_id
        self._excel_path = excel_path

    def on_enter(self, userdata):
        self._message = userdata.get_message

    def execute(self, userdata):
        try:
            if self.check_command_str() == CALL_COMMAND:
                userdata.pose_name = self.split_number(self._message)
                self.send_results_message(SEND_MESSAGE[1])
                return COMMAND_DICT[CALL_COMMAND]


            elif self.check_command_str() == CHARGE_COMMAND:
                userdata.pose_name = COMMAND_DICT[CHARGE_COMMAND]
                self.send_results_message(SEND_MESSAGE[2])
                return COMMAND_DICT[CHARGE_COMMAND]
            

            elif self.check_command_str() == PATROL_COMMAND:
                self.send_results_message(SEND_MESSAGE[3])
                return COMMAND_DICT[PATROL_COMMAND]

            else:
                self.send_results_message(SEND_MESSAGE[0])
                return COMMAND_DICT[FAIL_COMMAND]
        
        except Exception as e:
            Logger.loginfo(f"{e}")

    def check_command_str(self):
        if self._message.count(CALL_COMMAND) or self._message.count(DELIVERY_COMMAND):
            if self.split_number(self._message):
                if self.check_command_int(self.split_number(self._message)):
                    return CALL_COMMAND
        
        elif self._message.count(CHARGE_COMMAND):
            return CHARGE_COMMAND
                    
        elif self._message.count(PATROL_COMMAND):
            return PATROL_COMMAND
        
        else:
            return

    def check_command_int(self, value):
        df = pd.read_excel(self._excel_path)
        value_exists = int(value) in df['pose_name'].values
        return value_exists
    

    def split_number(self, text):
        numbers =re.findall(r'\d+', text)
        if len(numbers):
            return numbers[0]
        else:
            return False
        

    def send_results_message(self, message_text):
        bot = telebot.TeleBot(self._token)
        bot.send_message(chat_id=self._chat_id, text=message_text)