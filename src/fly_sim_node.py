#!/usr/bin/python

# Version: 1.1

import rospy
import os
from read_settings import *
from clientclass import BotClient
from move_base_msgs.msg import *
from geometry_msgs.msg import PoseStamped
from pose_dict_tf import *
from tf.transformations import euler_from_quaternion, quaternion_from_euler

RATE = 10


class FlyNode:

    ######################################################
    #  Initialise client with server address HOST_ADDR and
    #  map file directory MAP_DIR.
    def __init__(self, HOST_ADDR, MAP_DIR):
        self.FLY_CLIENT = BotClient(HOST_ADDR, 'FLYBOT')
        self.MAP_DIR = MAP_DIR
        self.PRINT_FLAG = None
        if(os.path.isfile(self.MAP_DIR + '/map.pgm')):
            os.remove(self.MAP_DIR + '/map.pgm')
        if(os.path.isfile(self.MAP_DIR + '/map.yaml')):
            os.remove(self.MAP_DIR + '/map.yaml')
        self.MAP_FLAG = None
        self.RVIZ_POSE = rospy.Subscriber('/move_base_simple/goal',
                                          PoseStamped, self.PoseCallback)

    ####################################################
    #  Uploads object pose to server.
    def PoseCallback(self, DATA):
        POSE_DICT = PoseStampedToDict(DATA)
        self.FLY_CLIENT.SendObjPose(str(POSE_DICT))

    ####################################################
    #  Work callback, executed at RATE (defined at top).
    def WorkCallback(self):
        CUR_SERV_STATE = self.FLY_CLIENT.ServState()
        #  Display current server state (if updated)
        if(self.PRINT_FLAG != CUR_SERV_STATE):
            self.PRINT_FLAG = CUR_SERV_STATE
            rospy.loginfo(self.PRINT_FLAG)
        #  If no map present, check server state for MAP_AT_SERVER
        #  and download to MAP_DIR if it's available from server.
        if(CUR_SERV_STATE == 'MAP_AT_SERVER'):
            self.MAP_FLAG = os.path.isfile(self.MAP_DIR + '/map.pgm') and \
                os.path.isfile(self.MAP_DIR + '/map.yaml')
            if(not self.MAP_FLAG):
                self.FLY_CLIENT.RecvMap(self.MAP_DIR)
                self.MAP_FLAG = True
        #  Reset map flag variable, removing existing
        #  files on reset.
        elif(CUR_SERV_STATE == 'RESET'):
            if(os.path.isfile(self.MAP_DIR + '/map.pgm')):
                os.remove(self.MAP_DIR + '/map.pgm')
            if(os.path.isfile(self.MAP_DIR + '/map.yaml')):
                os.remove(self.MAP_DIR + '/map.yaml')
            self.MAP_FLAG = None

    ####################################################
    #  Spin function starts/stops FLY client and 
    #  executes Work Callback at desired RATE.
    def Spin(self):
        rate = rospy.Rate(RATE)
        self.FLY_CLIENT.Start()
        while not rospy.is_shutdown():
            self.WorkCallback()
            rate.sleep()
        self.FLY_CLIENT.Stop()

if __name__ == '__main__':
    rospy.init_node('fly_sim_node')
    IP, MAP_DIR = ReadSettings('FLYBOT')
    PORT = int(raw_input('Enter port:\n'))
    HOST_ADDR = (IP, PORT)
    FLY = FlyNode(HOST_ADDR, MAP_DIR)
    FLY.Spin()
