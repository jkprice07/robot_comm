#!/usr/bin/python
#
#  Jason Price
#  Version: 2.4

import socket
from time import time, sleep
import threading
import logging


class BotClient:

    #########################################################
    #  Initialise client with server address and robot type.
    def __init__(self, HOST_ADDR, BOT_TYPE):
        self.HOST_ADDR = HOST_ADDR
        self.BOT_TYPE = BOT_TYPE
        self.STATE = 'NONE'
        self.POSE = 'NONE'
        self.SERV_STATE = 'NONE'
        self.SERV_SYNC = False
        self.STATE_TIME = 0
        self.POSE_TIME = 0

    #########################################################
    #  Starts threads for `Sync' and `DataTimeout' functions
    def Start(self):
        self.SERV_SYNC = True
        self.CLIENT_CONN = threading.Thread(target=self.Sync)
        self.CLIENT_CONN.start()
        self.TIMEOUT_THREAD = threading.Thread(target=self.DataTimeout)
        self.TIMEOUT_THREAD.start()

    #########################################################
    #  Ends current threads using `self.SERV_SYNC' variable.
    def Stop(self):
        self.SERV_SYNC = False
        self.SERV_STATE = 'NONE'

    #########################################################
    #  While sync'd update robot pose/state to server
    #  and request server state.
    def Sync(self):
        logging.info('Synchronization started.')
        while(self.SERV_SYNC):
            SOCK = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            SOCK.setblocking(1)
            SOCK.settimeout(5)
            try:
                SOCK.connect(self.HOST_ADDR)
                DATA = self.BOT_TYPE + '#' + \
                    self.STATE + '#' + self.POSE + '#'
                self.Send(SOCK, 1, DATA)
                self.SERV_STATE = self.Recv(SOCK, 32)
                sleep(0.2)
                SOCK.close()
            except socket.timeout:
                logging.info('Cannot connect, re-establishing \
                    connection to server...')
                sleep(0.2)
        logging.info('Synchronization stopped.')

    #########################################################
    #  Resets POSE and STATE variables after a timeout period
    #  of no updates (TIMEOUT set to 5 seconds).
    def DataTimeout(self):
        TIMEOUT = 5
        while(self.SERV_SYNC):
            CUR_TIME = time()
            if(CUR_TIME > (self.STATE_TIME + TIMEOUT)):
                self.STATE = 'NONE'
            if(CUR_TIME > (self.POSE_TIME + TIMEOUT)):
                self.POSE = 'NONE'
            sleep(1)

    #############################################
    #  Receive data from socket connection CONN,
    #  using packet size PACK_SIZE.
    def Recv(self, CONN, PACK_SIZE):
        DATA = ''
        while(True):
            CHUNK = CONN.recv(PACK_SIZE)
            if(CHUNK):
                DATA = DATA + CHUNK
            else:
                break
        return DATA

    #############################################
    #  Receive data from socket connection CONN,
    #  using packet size PACK_SIZE and save to
    #  file specified by PATH.
    def RecvFile(self, CONN, PACK_SIZE, PATH):
        FILE = open(PATH, 'wb')
        while(True):
            CHUNK = CONN.recv(PACK_SIZE)
            if(CHUNK):
                FILE.write(CHUNK)
            else:
                break
        FILE.close()
        CONN.close()

    ################################################
    #  Depending on TARGET, requests and retrieves
    #  pose string for ARMBOT or OBJECT.
    def RecvPose(self, TARGET):
        SOCK = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        SOCK.connect(self.HOST_ADDR)
        if(TARGET == 'OBJ'):
            SOCK.send('SEND_OBJ_POSE#')
        elif(TARGET == 'ARM'):
            SOCK.send('SEND_ARM_POSE#')
        POSE = self.Recv(SOCK, 32)
        SOCK.close()
        if(POSE):
            return POSE
        else:
            return None

    ##################################################
    #  Receive 2 map files, `map.pgm' and `map.yaml',
    #  saving in directory specified by DEST.  Using
    #  threads to avoid holding up program.
    def RecvMap(self, DEST):
        SOCK_0 = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        SOCK_0.connect(self.HOST_ADDR)
        SOCK_0.send('SEND_MAP_PGM#')
        THREAD_0 = threading.Thread(target=self.RecvFile,
                                    args=(SOCK_0, 32, DEST + '/map.pgm'))
        THREAD_0.start()
        SOCK_1 = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        SOCK_1.connect(self.HOST_ADDR)
        SOCK_1.send('SEND_MAP_YAML#')
        THREAD_1 = threading.Thread(target=self.RecvFile,
                                    args=(SOCK_1, 32, DEST + '/map.yaml'))
        THREAD_1.start()

    #################################
    #  Send input DATA of type TYPE.
    def SendFile(self, DATA, TYPE):
        if(TYPE == 'MAP_YAML'):
            MSG = 'RECV_MAP_YAML#'
        elif(TYPE == 'MAP_PGM'):
            MSG = 'RECV_MAP_PGM#'
        elif(TYPE == 'OBJ'):
            MSG = 'RECV_OBJ_IM#'
        elif(TYPE == 'VER'):
            MSG = 'RECV_VER_IM#'
        THREAD = threading.Thread(target=self.Send,
                                  args=(None, 32, MSG + DATA))
        THREAD.start()

    ##############################################
    #  Send pose of object (input `PoseStamped()
    #  dictionary converted to string and transmitted).
    def SendObjPose(self, POSE):
        self.Send(None, 32, 'RECV_OBJ_POSE#' + str(POSE))

    ##############################################
    #  Send input DATA to socket connection CONN,
    #  using packet size PACK_SIZE.
    def Send(self, CONN, PACK_SIZE, DATA):
        SIZE = len(DATA)
        PACK = 0
        INDEX = 0
        SENT = 0
        if(not CONN):
            CONN = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            CONN.connect(self.HOST_ADDR)
            CONN.setblocking(1)
        while(True):
            INDEX = PACK * 1
            CHUNK = DATA[INDEX:INDEX + PACK_SIZE]
            CONN.send(CHUNK)
            SENT = SENT + len(CHUNK)
            PACK = PACK + PACK_SIZE
            if(SENT == SIZE):
                break

    def SetState(self, STATE):
        self.STATE = STATE
        self.STATE_TIME = time()

    def SetPose(self, POSE):
        self.POSE = POSE
        self.POSE_TIME = time()

    def ServState(self):
        return self.SERV_STATE

    def CurState(self):
        return self.STATE

if __name__ == "__main__":
    pass
