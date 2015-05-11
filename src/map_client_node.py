#!/usr/bin/python

# Version: 2.1

import rospy
import subprocess
import os
import time
from read_settings import *
from tf.transformations import euler_from_quaternion
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped
from clientclass import BotClient
from pose_dict_tf import *

RATE = 2


class MapClientNode:

    ######################################################
    #  Initialise client with server address HOST_ADDR and
    #  map file directory MAP_DIR.
    def __init__(self, HOST_ADDR, MAP_DIR):
        self.HOST_ADDR = HOST_ADDR
        self.MAP_DIR = MAP_DIR
        self.MAP_CLIENT = BotClient(HOST_ADDR, 'MAPBOT')
        self.PRINT_FLAG = None
        # if(os.path.isfile(self.MAP_DIR + '/map.pgm')):
        #    os.remove(self.MAP_DIR + '/map.pgm')
        # if(os.path.isfile(self.MAP_DIR + '/map.yaml')):
        #    os.remove(self.MAP_DIR + '/map.yaml')
        self.MAP_FLAG = None
        #  Subscriptions for MAP state and MAP pose.
        rospy.Subscriber('/map_bot_base/state',
                         String,
                         self.MapStateCallback)
        rospy.Subscriber('/slam_out_pose',
                         PoseStamped,
                         self.MapPoseCallback)
        #  Publication for server state.
        self.SERV_STATE = rospy.Publisher('/client_node/serv_state', String)

    #######################################
    #  Updates MAP state in client.
    def MapStateCallback(self, DATA):
        self.MAP_CLIENT.SetState(DATA.data)

    ###############################################
    #  Updates MAP pose in client.  Converts pose
    #  to a dictionary, converts the data to euler
    #  co-ords from quaternion and send to server.
    def MapPoseCallback(self, DATA):
        POSE_DICT = PoseStampedToDict(DATA)
        POSE_DICT_STRING = str(QuaternionToEulerDict(POSE_DICT))
        self.MAP_CLIENT.SetPose(POSE_DICT_STRING)

    ####################################################
    #  Work callback, executed at RATE (defined at top).
    def WorkCallback(self):
        #  Publish current server state.
        CUR_SERV_STATE = self.MAP_CLIENT.ServState()
        self.SERV_STATE.publish(CUR_SERV_STATE)
        #  Display current server state (if updated).
        if(self.PRINT_FLAG != CUR_SERV_STATE):
            self.PRINT_FLAG = CUR_SERV_STATE
            rospy.loginfo(self.PRINT_FLAG)
        #  When server state is MAP_DONE, the mapping is stopped,
        #  the map is saved, the `.yaml' file is amended and both
        #  the `.pgm' and modified `.yaml' are sent to the server.
        if(CUR_SERV_STATE == 'MAP_DONE'):
            if(not self.MAP_FLAG):
                #subprocess.call('rosrun map_server \
                    #map_saver -f map',  shell=True)
                YAML_DATA = self.ReadFile(self.MAP_DIR + '/map.yaml')
                YAML_DATA = self.FixYAML(YAML_DATA)
                self.MAP_CLIENT.SendFile(YAML_DATA, 'MAP_YAML')
                PGM_DATA = self.ReadFile(self.MAP_DIR + '/map.pgm')
                self.MAP_CLIENT.SendFile(PGM_DATA, 'MAP_PGM')
                self.MAP_FLAG = True
        else:
            #self.MAP_FLAG = os.path.isfile(self.MAP_DIR + '/map.pgm') and \
                #os.path.isfile(self.MAP_DIR + '/map.yaml')
            self.MAP_FLAG = None

    #######################################################
    #  Amends first line of `.yaml' DATA to fix image name.
    def FixYAML(self, DATA):
        if(DATA):
            i = 0
            while(True):
                CHAR = DATA[i]
                i = i + 1
                if(CHAR == '\n'):
                    break
            return 'image: map.pgm\n' + DATA[i:len(DATA)]
        else:
            return None

    ##################################################
    #  Reads binary data from file specified by PATH.
    def ReadFile(self, PATH):
        try:
            FILE = open(PATH, 'rb')
            DATA = FILE.read()
            FILE.close()
            return DATA
        except:
            return None

    ###############################################
    #  Spin function starts/stops MAP client and
    #  executes Work Callback at desired RATE.
    def Spin(self):
        self.MAP_CLIENT.Start()
        rate = rospy.Rate(RATE)
        while not rospy.is_shutdown():
            self.WorkCallback()
            rate.sleep()
        self.MAP_CLIENT.Stop()

if __name__ == "__main__":
    rospy.init_node('map_client_node')
    IP, MAP_DIR = ReadSettings('MAPBOT')
    PORT = int(raw_input('Enter port:\n'))
    HOST_ADDR = (IP, PORT)
    CLIENT_NODE = MapClientNode(HOST_ADDR, MAP_DIR)
    CLIENT_NODE.Spin()
