import socket
import os
from read_settings import *


IP, MAP, IM = ReadSettings('SERVER')
PORT = int(raw_input('Enter port:\n'))
HOST_ADDR = (IP, PORT)
while(True):
    print 'Enter 1 to START,\n\
        2 to signal MAP completion,\n\
        3 to decide on PICKUP,\n\
        4 to VERIFY successful pickup,\n\
        5 to RESET,\n\
        6 to QUIT.'
    ACTION = raw_input('')
    if(ACTION == '1'):
        sock = socket.socket()
        sock.connect(HOST_ADDR)
        sock.send('START#')
    elif(ACTION == '2'):
        sock = socket.socket()
        sock.connect(HOST_ADDR)
        sock.send('INPUT#TRUE#')
    elif(ACTION == '3'):
        print 'Enter 1 to PICKUP,\n2 to LEAVE.'
        ACTION = raw_input('')
        if(ACTION == '1'):
            sock = socket.socket()
            sock.connect(HOST_ADDR)
            sock.send('INPUT#PICKUP#')
        elif(ACTION == '2'):
            sock = socket.socket()
            sock.connect(HOST_ADDR)
            sock.send('INPUT#LEAVE#')
    elif(ACTION == '4'):
        print 'Enter 1 for success,\n2 for failure.'
        ACTION = raw_input('')
        if(ACTION == '1'):
            sock = socket.socket()
            sock.connect(HOST_ADDR)
            sock.send('INPUT#SUCC#')
        elif(ACTION == '2'):
            sock = socket.socket()
            sock.connect(HOST_ADDR)
            sock.send('INPUT#FAIL#')
    elif(ACTION == '5'):
        sock = socket.socket()
        sock.connect(HOST_ADDR)
        sock.send('RESET#')
    elif(ACTION == '6'):
        break
