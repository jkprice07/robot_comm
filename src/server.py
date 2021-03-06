#!/usr/bin/python
#
#  Server class, initialised with a FSM.
#
#  Jason Price
#  Version: 2.7

import socket
import threading
import ast
import os
from time import sleep, time
from PIL import Image
from read_settings import *
from server_fsm import *


class Server:

    ##################################################################
    #  Initialised with HOST_ADDR as tuple consisiting of {IP, PORT},
    #  MAP_DIR a string pointing to directory for map files,
    #  IMAGE_DIR a string pointing to directory for image files.
    def __init__(self, HOST_ADDR, MAP_DIR, IMAGE_DIR):
        #  Input variables
        self.HOST_ADDR = HOST_ADDR
        self.MAP_DIR = MAP_DIR
        self.IMAGE_DIR = IMAGE_DIR
        #  Initialise server FSM.
        self.InitServFSM()
        #  Set dictionary for bot data.
        self.DATA = {'STATES': {'ARMBOT': {'NONE'},
                                'BASEBOT': {'NONE'},
                                'BINBOT': {'NONE'},
                                'MAPEBOT': {'NONE'},
                                'FLYBOT': {'NONE'}},
                     'POSES': {'BASEBOT': 'NONE',
                               'BINBOT': 'NONE',
                               'MAPBOT': 'NONE',
                               'FLYBOT': 'NONE'}}
        #  Initialise server variables.
        self.START_PROCESS = False
        self.SYNC = False
        self.USER_INPUT = None
        self.COUNT = 0
        #  Initialise MAP and IMAGE flags.
        self.MAP_FLAG = None
        self.IMAGE_FLAG = None
        #  Initialise object location list.
        self.OBJ_POSE = []
        #  Initialise variables for bot data timeout.
        self.ARM_TIME = 0
        self.BASE_TIME = 0
        self.BIN_TIME = 0
        self.FLY_TIME = 0
        self.MAP_TIME = 0

    ##############################################################
    #  Initialises server finite-state-machine.  Adds all states
    #  and transitions to the FSM object and sets initial state.
    def InitServFSM(self):
        self.FSM = FSM()
        #  Add states.
        self.FSM.AddState('IDLE', IDLE(self))
        self.FSM.AddState('START', START(self))
        self.FSM.AddState('MAPPING_AUTO', MAPPING_AUTO(self))
        self.FSM.AddState('MAPPING_MAN', MAPPING_MAN(self))
        self.FSM.AddState('MAP_DONE', MAP_DONE(self))
        self.FSM.AddState('MAP_AT_SERVER', MAP_AT_SERVER(self))
        self.FSM.AddState('FINDING_OBJ', FINDING_OBJ(self))
        self.FSM.AddState('FOUND_OBJ', FOUND_OBJ(self))
        self.FSM.AddState('ARM_TO_OBJ', ARM_TO_OBJ(self))
        self.FSM.AddState('ARM_SEARCH', ARM_SEARCH(self))
        self.FSM.AddState('USER_DEC', USER_DEC(self))
        self.FSM.AddState('BIN_TO_ARM', BIN_TO_ARM(self))
        self.FSM.AddState('BIN_AT_ARM', BIN_AT_ARM(self))
        self.FSM.AddState('ARM_PICKUP', ARM_PICKUP(self))
        self.FSM.AddState('PICKUP_CHECK', PICKUP_CHECK(self))
        self.FSM.AddState('ARM_DROPPING', ARM_DROPPING(self))
        self.FSM.AddState('BIN_TO_BASE', BIN_TO_BASE(self))
        self.FSM.AddState('RESET', RESET(self))
        #  Add state transitions.
        self.FSM.AddTransition('To_IDLE', Transition('IDLE'))
        self.FSM.AddTransition('To_START', Transition('START'))
        self.FSM.AddTransition('To_MAPPING_AUTO', Transition('MAPPING_AUTO'))
        self.FSM.AddTransition('To_MAPPING_MAN', Transition('MAPPING_MAN'))
        self.FSM.AddTransition('To_MAP_DONE', Transition('MAP_DONE'))
        self.FSM.AddTransition('To_MAP_AT_SERVER', Transition('MAP_AT_SERVER'))
        self.FSM.AddTransition('To_FINDING_OBJ', Transition('FINDING_OBJ'))
        self.FSM.AddTransition('To_FOUND_OBJ', Transition('FOUND_OBJ'))
        self.FSM.AddTransition('To_ARM_TO_OBJ', Transition('ARM_TO_OBJ'))
        self.FSM.AddTransition('To_ARM_SEARCH', Transition('ARM_SEARCH'))
        self.FSM.AddTransition('To_USER_DEC', Transition('USER_DEC'))
        self.FSM.AddTransition('To_BIN_TO_ARM', Transition('BIN_TO_ARM'))
        self.FSM.AddTransition('To_BIN_AT_ARM', Transition('BIN_AT_ARM'))
        self.FSM.AddTransition('To_ARM_PICKUP', Transition('ARM_PICKUP'))
        self.FSM.AddTransition('To_PICKUP_CHECK', Transition('PICKUP_CHECK'))
        self.FSM.AddTransition('To_ARM_DROPPING', Transition('ARM_DROPPING'))
        self.FSM.AddTransition('To_BIN_TO_BASE', Transition('BIN_TO_BASE'))
        self.FSM.AddTransition('To_RESET', Transition('RESET'))
        #  Set inital state.
        self.FSM.SetState('IDLE')

    def FSMExecute(self):
        self.FSM.Execute()

    ##############################################################
    #  `Run' enables the server to listen for incoming connections
    #  on one thread (`Listen') and spins `DataTimeout' on another.
    def Run(self):
        self.SYNC = True
        self.SERV_THREAD = threading.Thread(target=self.Listen)
        self.SERV_THREAD.start()
        self.DATA_THREAD = threading.Thread(target=self.DataTimeout)
        self.DATA_THREAD.start()

    #########################################################
    #  After a maximum of 3 seconds, closes the server (due
    #  to socket timeout).
    def Stop(self):
        self.SYNC = False

    ######################################################################
    #  Listens for incoming connections and assigns socket connections for
    #  client-server communication (`ClientConn' thread started for each
    #  client).
    def Listen(self):
        SOCK = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        SOCK.bind(self.HOST_ADDR)
        SOCK.listen(15)
        SOCK.settimeout(3)
        while(self.SYNC):
            try:
                CONN, ADDR = SOCK.accept()
                CONN.setblocking(1)
                CLIENT_THREAD = threading.Thread(
                    target=self.ClientConn, args=(CONN, ADDR))
                CLIENT_THREAD.start()
            except socket.timeout:
                pass
        SOCK.close()
        print 'Socket closed.'

    #####################################################################
    #  Message handling for the server.  Each client spawns a thread of
    #  `ClientConn' to read the socket data and respond appropriately.
    def ClientConn(self, CONN, ADDR):
        CONN.settimeout(15)
        try:
            MSG = self.RecvLine(CONN)

            ##############################################################
            #  Message handlers for robot state/pose and server state data
            #  transfer.  Records time of message for use in `DataTimeout'.
            if(MSG == 'ARMBOT'):
                self.DATA['STATES']['ARMBOT'] = self.RecvLine(CONN)
                self.RecvLine(CONN)
                self.Send(CONN, 32, self.FSM.curState.StateName())
                self.ARM_TIME = time()

            elif(MSG == 'BASEBOT'):
                self.DATA['STATES']['BASEBOT'] = self.RecvLine(CONN)
                POSE = self.RecvLine(CONN)
                if(POSE != 'NONE'):
                    self.DATA['POSES']['BASEBOT'] = ast.literal_eval(POSE)
                self.Send(CONN, 32, self.FSM.curState.StateName())
                self.BASE_TIME = time()

            elif(MSG == 'MAPBOT'):
                self.DATA['STATES']['MAPBOT'] = self.RecvLine(CONN)
                POSE = self.RecvLine(CONN)
                if(POSE != 'NONE'):
                    self.DATA['POSES']['MAPBOT'] = ast.literal_eval(POSE)
                self.Send(CONN, 32, self.FSM.curState.StateName())
                self.MAP_TIME = time()

            elif(MSG == 'BINBOT'):
                self.DATA['STATES']['BINBOT'] = self.RecvLine(CONN)
                POSE = self.RecvLine(CONN)
                if(POSE != 'NONE'):
                    self.DATA['POSES']['BINBOT'] = ast.literal_eval(POSE)
                self.Send(CONN, 32, self.FSM.curState.StateName())
                self.BIN_TIME = time()

            elif(MSG == 'FLYBOT'):
                self.DATA['STATES']['FLYBOT'] = self.RecvLine(CONN)
                POSE = self.RecvLine(CONN)
                if(POSE != 'NONE'):
                    self.DATA['POSES']['FLYBOT'] = ast.literal_eval(POSE)
                self.Send(CONN, 32, self.FSM.curState.StateName())
                self.FLY_TIME = time()
                
            elif(MSG == 'IPAD'):
                self.Send(CONN, 32, self.FSM.curState.StateName())

            ###################################################
            #  Message handler for receiving object pose (i.e.
            #  location of object) and sending object/armbot pose.
            elif(MSG == 'RECV_OBJ_POSE'):
                self.OBJ_POSE.append(ast.literal_eval(self.Recv(CONN, 32)))
                print type(self.OBJ_POSE[0]), self.OBJ_POSE

            elif(MSG == 'SEND_OBJ_POSE'):
                if(len(self.OBJ_POSE)):
                    self.Send(CONN, 32, str(self.OBJ_POSE[0]))

            elif(MSG == 'SEND_ARM_POSE'):
                if(isinstance(self.DATA['POSES']['BASEBOT'], dict)):
                    self.Send(CONN, 32, str(self.DATA['POSES']['BASEBOT']))

            ##########################################################
            #  Message handlers for receiving files (map and images).
            elif(MSG == 'RECV_OBJ_IM'):
                self.RecvFile(CONN, 32, self.IMAGE_DIR + '/object_image.ppm')
                FILE = Image.open(self.IMAGE_DIR + '/object_image.ppm')
                FILE.save(self.IMAGE_DIR + '/object_image.png')

            elif(MSG == 'RECV_VER_IM'):
                self.RecvFile(CONN, 32, self.IMAGE_DIR + '/verify_image.ppm')
                FILE = Image.open(self.IMAGE_DIR + '/verify_image.ppm')
                FILE.save(self.IMAGE_DIR + '/verify_image.png')

            elif(MSG == 'RECV_MAP_PGM'):
                self.RecvFile(CONN, 32, self.MAP_DIR + '/map.pgm')
                FILE = Image.open(self.MAP_DIR + '/map.pgm')
                FILE.save(self.MAP_DIR + '/map.png')

            elif(MSG == 'RECV_MAP_YAML'):
                self.RecvFile(CONN, 32, self.MAP_DIR + '/map.yaml')

            ##############################################################
            #  Message handlers for sending map, object detection and
            #  verification PNGs to iPad and map files to robot client
            #  (.pgm + .yaml).
            elif(MSG == 'SEND_OBJ_PNG'):
                if(self.IMAGE_FLAG):
                    DATA = self.ReadFile(self.IMAGE_DIR + '/object_image.png')
                    self.Send(CONN, 1024, DATA)

            elif(MSG == 'SEND_VER_PNG'):
                if(self.IMAGE_FLAG):
                    DATA = self.ReadFile(self.IMAGE_DIR + '/verify_image.png')
                    self.Send(CONN, 1024, DATA)

            elif(MSG == 'SEND_MAP_PNG'):
                if(os.path.isfile(self.MAP_DIR + '/map.png')):
                    DATA = self.ReadFile(self.MAP_DIR + '/map.png')
                    print len(DATA)
                    self.Send(CONN, 1024, DATA)
                elif(os.path.isfile(self.MAP_DIR + '/map.pgm')):
                    FILE = Image.open(self.MAP_DIR + '/map.pgm')
                    FILE.save(self.MAP_DIR + '/map.png')
                    DATA = self.ReadFile(self.MAP_DIR + '/map.png')
                    self.Send(CONN, 1024, DATA)

            elif(MSG == 'SEND_MAP_PGM'):
                if(self.MAP_FLAG):
                    DATA = self.ReadFile(self.MAP_DIR + '/map.pgm')
                    self.Send(CONN, 32, DATA)

            elif(MSG == 'SEND_MAP_YAML'):
                if(self.MAP_FLAG):
                    DATA = self.ReadFile(self.MAP_DIR + '/map.yaml')
                    self.Send(CONN, 32, DATA)

            ##############################################################
            #  Message handlers for starting and reseting the process,
            #  accepting user input from the iPad and sending robot
            #  state/pose data to the iPad.
            elif(MSG == 'START'):
                self.START_PROCESS = True

            elif(MSG == 'RESET'):
                self.SetCount()
                self.FSM.SetState('RESET')

            elif(MSG == 'INPUT'):
                self.USER_INPUT = self.RecvLine(CONN)

            elif(MSG == 'SEND_STATES'):
                CONN.send('STATE#' + self.DATA['STATES']['ARMBOT'] + '#' +
                          self.DATA['STATES']['BASEBOT'] + '#' +
                          self.DATA['STATES']['BINBOT'] + '#' +
                          self.DATA['STATES']['FLYBOT'] + '#' +
                          self.DATA['STATES']['MAPBOT'] + '#' +
                          self.FSM.curState.StateName() + '#')

            elif(MSG == 'SEND_POSES'):
                BASE_X, BASE_Y, BASE_ANG = self.ReadPoseDict('BASEBOT')
                BIN_X, BIN_Y, BIN_ANG = self.ReadPoseDict('BINBOT')
                MAP_X, MAP_Y, MAP_ANG = self.ReadPoseDict('MAPBOT')
                CONN.send('POSE#' + BASE_X + '#' + BASE_Y + '#' +
                          BASE_ANG + '#' + BIN_X + '#' + BIN_Y +
                          '#' + BIN_ANG + '#' + MAP_X +
                          '#' + MAP_Y + '#' + MAP_ANG + '#')
            else:
                print 'Unknown message: ', MSG
            CONN.close()
        except socket.timeout:
            try:
                CONN.close()
            except:
                print 'Client socket error'

    ##############################################################
    #  Reset robot data variables after TIMEOUT seconds of no new
    #  data.  Check and set IMAGE and MAP flags, allow user input
    #  during allowed server states (USER_INPUT).
    def DataTimeout(self):
        while(self.SYNC):
            #  Check if data is less than TIMEOUT seconds old,
            #  reseting to `NONE' if so.
            CUR_TIME = time()
            TIMEOUT = 5
            CUR_STATE = self.FSM.curState.StateName()
            if(CUR_TIME > (self.ARM_TIME + TIMEOUT)):
                self.DATA['STATES']['ARMBOT'] = 'NONE'
            if(CUR_TIME > (self.BASE_TIME + TIMEOUT)):
                self.DATA['STATES']['BASEBOT'] = 'NONE'
                self.DATA['POSES']['BASEBOT'] = 'NONE'
            if(CUR_TIME > (self.FLY_TIME + TIMEOUT)):
                self.DATA['STATES']['FLYBOT'] = 'NONE'
                self.DATA['POSES']['FLYBOT'] = 'NONE'
            if(CUR_TIME > (self.BIN_TIME + TIMEOUT)):
                self.DATA['STATES']['BINBOT'] = 'NONE'
                self.DATA['POSES']['BINBOT'] = 'NONE'
            if(CUR_TIME > (self.MAP_TIME + TIMEOUT)):
                self.DATA['STATES']['MAPBOT'] = 'NONE'
                self.DATA['POSES']['MAPBOT'] = 'NONE'
            #  Set map flag (check for .pgm and .yaml).
            CON_1 = os.path.isfile(self.MAP_DIR + '/map.pgm')
            CON_2 = os.path.isfile(self.MAP_DIR + '/map.yaml')
            self.MAP_FLAG = CON_1 and CON_2
            #  Set image flag (check for png's in image dir).
            CON_1 = os.path.isfile(self.IMAGE_DIR + '/object_image.png')
            CON_2 = os.path.isfile(self.IMAGE_DIR + '/verify_image.png')
            self.IMAGE_FLAG = CON_1 or CON_2
            #  Remove image files/reset user input if not in decision state.
            if((CUR_STATE != 'ARM_SEARCH') and (CUR_STATE != 'USER_DEC')):
                if(os.path.isfile(self.IMAGE_DIR + '/object_image.png')):
                    os.remove(self.IMAGE_DIR + '/object_image.ppm')
                    os.remove(self.IMAGE_DIR + '/object_image.png')
            if((CUR_STATE != 'ARM_PICKUP') and (CUR_STATE != 'PICKUP_CHECK')):
                if(os.path.isfile(self.IMAGE_DIR + '/verify_image.png')):
                    os.remove(self.IMAGE_DIR + '/verify_image.ppm')
                    os.remove(self.IMAGE_DIR + '/verify_image.png')
            if((CUR_STATE != 'USER_DEC') and (CUR_STATE != 'PICKUP_CHECK') and
                    (CUR_STATE != 'MAPPING_MAN')):
                self.USER_INPUT = None
            #  Reset pose list and start variable in RESET.
            if(CUR_STATE == 'RESET'):
                self.START_PROCESS = False
                self.OBJ_POSE = []
            sleep(1)

    ####################################################
    #  Functions to return server variables.  Used
    #  inside state objects to access server variables.
    def UserData(self):
        return self.USER_INPUT

    def MapFlag(self):
        return self.MAP_FLAG

    def ImageFlag(self):
        return self.IMAGE_FLAG

    def ObjPose(self):
        return self.OBJ_POSE

    def BotData(self):
        return self.DATA

    def StartProcess(self):
        return self.START_PROCESS

    def ViewCount(self):
        return self.COUNT

    def SetCount(self):
        self.COUNT = time()

    ################################################
    #  Pops the first item from the OBJ_POSE list,
    #  used to indicate an objects retreival.
    def PopObjPose(self):
        if(self.OBJ_POSE):
            self.OBJ_POSE.reverse()
            self.OBJ_POSE.pop()
            self.OBJ_POSE.reverse()

    ################################################
    #  Reads binary data from file specified by PATH.
    def ReadFile(self, PATH):
        try:
            FILE = open(PATH, 'rb')
            DATA = FILE.read()
            FILE.close()
            return DATA
        except:
            return None

    ##########################################
    #  Return current state of robots/server.
    def StateData(self):
        return self.DATA['STATES']['ARMBOT'],\
            self.DATA['STATES']['BASEBOT'],\
            self.DATA['STATES']['BINBOT'],\
            self.DATA['STATES']['FLYBOT'],\
            self.DATA['STATES']['MAPBOT'],\
            self.FSM.curState.StateName()

    ###########################################
    #  Return X, Y and YAW of robots as string.
    def ReadPoseDict(self, BOT):
        if(isinstance(self.DATA['POSES'][BOT], dict)):
            X = str(self.DATA['POSES'][BOT]['pose']['position']['x'])
            Y = str(self.DATA['POSES'][BOT]['pose']['position']['y'])
            ANG_RAD = self.DATA['POSES'][BOT]['pose']['orientation']['yaw']
            ANG_DEG = str((ANG_RAD + 3.14) * 180 / 3.14)
        else:
            X = 'NONE'
            Y = 'NONE'
            ANG_DEG = 'NONE'
        return X, Y, ANG_DEG

    ##############################################
    #  Send input DATA to socket connection CONN,
    #  using packet size PACK_SIZE.
    def Send(self, CONN, PACK_SIZE, DATA):
        SIZE = len(DATA)
        PACK = 0
        INDEX = 0
        SENT = 0
        while(True):
            INDEX = PACK * 1
            CHUNK = DATA[INDEX:INDEX + PACK_SIZE]
            CONN.send(CHUNK)
            SENT = SENT + len(CHUNK)
            PACK = PACK + PACK_SIZE
            if(SENT == SIZE):
                break

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
    # until end-of-line character `#'.
    def RecvLine(self, CONN):
        DATA = ''
        while(True):
            CHAR = CONN.recv(1)
            if(len(CHAR) == 1):
                if(CHAR == '#'):
                    break
                else:
                    DATA = DATA + CHAR
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

if __name__ == "__main__":
    IP, MAP_DIR, IMAGE_DIR = ReadSettings('SERVER')
    PORT = int(raw_input('Enter port:\n'))
    HOST_ADDR = (IP, PORT)
    SERV = Server(HOST_ADDR, MAP_DIR, IMAGE_DIR)
    SERV.Run()
    sleep(2)
    try:
        while(True):
            SERV.FSMExecute()
            BASE_X, BASE_Y, BASE_ANG = SERV.ReadPoseDict('BASEBOT')
            BIN_X, BIN_Y, BIN_ANG = SERV.ReadPoseDict('BINBOT')
            MAP_X, MAP_Y, MAP_ANG = SERV.ReadPoseDict('MAPBOT')
            ARM, BASE, BIN, FLY, MAP, SERV_STATE = SERV.StateData()
            print('\nTime: ' + str(time()))
            print('Server state: ' + SERV_STATE)
            print('Arm state:    ' + ARM)
            print('Base state:   ' + BASE)
            print('Arm pose:   x:' + BASE_X + ', y: ' +
                  BASE_Y + ', ang: ' + BASE_ANG)
            print('Bin state:    ' + BIN)
            print('Bin pose:   x:' + BIN_X + ', y: ' +
                  BIN_Y + ', ang: ' + BIN_ANG)
            print('Map state:    ' + MAP)
            print('Map pose:   x:' + MAP_X + ', y: ' +
                  MAP_Y + ', ang: ' + MAP_ANG)
            sleep(0.5)
    except KeyboardInterrupt:
        SERV.Stop()
