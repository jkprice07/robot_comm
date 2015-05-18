#!/usr/bin/python

# Version: 2.0

import rospy
import time
import threading
import ast
import logging
import os
from read_settings import *
from tf.transformations import euler_from_quaternion
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from clientclass import BotClient
from pose_dict_tf import *

RATE = 5


class BinClientNode:

    ######################################################
    #  Initialise client with server address HOST_ADDR and
    #  map file directory MAP_DIR.  Any exisiting map files
    #  are deleted on startup.
    def __init__(self, HOST_ADDR, MAP_DIR):
        self.HOST_ADDR = HOST_ADDR
        self.MAP_DIR = MAP_DIR
        self.BIN_CLIENT = BotClient(HOST_ADDR, 'BINBOT')
        self.PRINT_FLAG = None
        self.POSE_FLAG = None
        if(os.path.isfile(self.MAP_DIR + '/map.pgm')):
            os.remove(self.MAP_DIR + '/map.pgm')
        if(os.path.isfile(self.MAP_DIR + '/map.yaml')):
            os.remove(self.MAP_DIR + '/map.yaml')
        self.MAP_FLAG = None
        #  Subscriptions for BIN state and BIN pose.
        rospy.Subscriber('/bin_bot_base/state',
                         String,
                         self.BinStateCallback)
        rospy.Subscriber('/amcl_pose',
                         PoseWithCovarianceStamped,
                         self.BinPoseCallback)
        #  Publications for server state and object pose goal.
        self.POSE_TOPIC = rospy.Publisher('/bin_bot_base/goal',
                                          PoseStamped)
        self.SERV_STATE = rospy.Publisher('/client_node/serv_state',
                                          String)

    #######################################
    #  Updates BIN state in client.
    def BinStateCallback(self, DATA):
        self.BIN_CLIENT.SetState(DATA.data)

    ###############################################
    #  Updates BIN pose in client.  Converts pose
    #  to a dictionary, converts the data to euler
    #  co-ords from quaternion and send to server.
    def BinPoseCallback(self, DATA):
        POSE_DICT = PoseCovarianceToDict(DATA)
        POSE_DICT_STRING = str(QuaternionToEulerDict(POSE_DICT))
        self.BIN_CLIENT.SetPose(POSE_DICT_STRING)

    ####################################################
    #  Work callback, executed at RATE (defined at top).
    def WorkCallback(self):
        #  Publish current server state.
        CUR_SERV_STATE = self.BIN_CLIENT.ServState()
        self.SERV_STATE.publish(CUR_SERV_STATE)
        #  Display current server state (if updated).
        if(self.PRINT_FLAG != CUR_SERV_STATE):
            self.PRINT_FLAG = CUR_SERV_STATE
            rospy.loginfo(self.PRINT_FLAG)
        #  When object is to be retrieved (server state BIN_TO_ARM),
        #  an object pose is downloaded, converted to PoseStamped()
        #  message type and published to necessary ROS topic.
        if(CUR_SERV_STATE == 'BIN_TO_ARM'):
            if(not self.POSE_FLAG):
                POSE_STRING = self.BIN_CLIENT.RecvPose('OBJ')
                rospy.loginfo('POSE: ' + POSE_STRING)
                if(isinstance(POSE_STRING, str)):
                    POSE_DICT = ast.literal_eval(POSE_STRING)
                    POSE_STAMPED = DictToPoseStamped(POSE_DICT)
                    self.POSE_TOPIC.publish(POSE_STAMPED)
                    self.POSE_FLAG = True
        else:
            self.POSE_FLAG = None
        #  If no map present, check server state for MAP_AT_SERVER
        #  and download to MAP_DIR if it's available from server.
        if(CUR_SERV_STATE == 'MAP_AT_SERVER'):
            if(not self.MAP_FLAG):
                self.BIN_CLIENT.RecvMap(self.MAP_DIR)
                self.MAP_FLAG = True
        else:
            self.MAP_FLAG = os.path.isfile(self.MAP_DIR + '/map.pgm') and \
                os.path.isfile(self.MAP_DIR + '/map.yaml')

    ####################################################
    #  Spin function starts/stops BIN client and 
    #  executes Work Callback at desired RATE.
    def Spin(self):
        self.BIN_CLIENT.Start()
        rate = rospy.Rate(RATE)
        while not rospy.is_shutdown():
            self.WorkCallback()
            rate.sleep()
        self.BIN_CLIENT.Stop()

if __name__ == "__main__":
    rospy.init_node('bin_client_node')
    IP, MAP_DIR = ReadSettings('BINBOT')
    PORT = int(raw_input('Enter port:\n'))
    HOST_ADDR = (IP, PORT)
    CLIENT_NODE = BinClientNode(HOST_ADDR, MAP_DIR)
    CLIENT_NODE.Spin()
