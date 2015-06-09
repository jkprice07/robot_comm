#!/usr/bin/python

# Version: 1.0

import os
import threading
import time
from read_settings import *
from clientclass import BotClient


class Ipad:

    def __init__(self, HOST_ADDR):
        self.CLIENT = BotClient(HOST_ADDR, '')

    def WorkCallback(self):
        SERV_STATE = self.CLIENT.ServState()
        if(SERV_STATE == 'USER_DEC'):
            self.CLIENT.Send(None , 32, 'INPUT#PICKUP#')
        elif(SERV_STATE == 'PICKUP_CHECK):
            self.CLIENT.Send(None , 32, 'INPUT#SUCC#')

    def Spin(self):
        self.CLIENT.Start()
        while(True):
            print '1 to START,\n\
                2 to signal MAP completion,\n\
                3 to RESET,\n\
                4 to QUIT.'
            ACTION = raw_input('')
            if(ACTION == '1'):
                self.CLIENT.Send(None , 32, 'START#')
            elif(ACTION == '2'):
                self.CLIENT.Send(None , 32, 'INPUT#TRUE#')
            elif(ACTION == '3'):
                sself.CLIENT.Send(None , 32, 'RESET#')
            elif(ACTION == '4'):
                break
            self.WorkCallback()
            time.sleep(1)
        self.CLIENT.Stop()

if __name__ == '__main__':
    IP, _ = ReadSettings('FLYBOT')
    PORT = int(raw_input('Enter port:\n'))
    HOST_ADDR = (IP, PORT)
    CLIENT = Ipad(HOST_ADDR)
    CLIENT.Spin()
