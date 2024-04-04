#!/usr/bin/python3

import telebot
from flexbe_core import EventState, Logger

class SendResults(EventState):
    def __init__(self, token, chat_id):

        super(SendResults, self).__init__(outcomes = ['next', 'fail'], 
                                          input_keys = ['results'])
        self._token = token
        self._chat_id = chat_id
        self._bot = telebot.TeleBot(self._token)

    def execute(self, userdata):
        try:
            self._bot.send_message(chat_id=self._chat_id, text=userdata.results)
            return 'next'
        except Exception as e:
            Logger.loginfo(f"{e}")
            return 'fail'