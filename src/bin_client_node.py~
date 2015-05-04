#!/usr/bin/python

# Version: 2.0

import rospy
import time
import threading
import ast
import logging
import os
from tf.transformations import euler_from_quaternion
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from clientclass import BotClient
from pose_dict_tf import *

RATE = 10


class BinClientNode:

    def __init__(self, HOST_ADDR, MAP_DIR):
        self.HOST_ADDR = HOST_ADDR
        self.MAP_DIR = MAP_DIR
        self.BIN_CLIENT = BotClient(HOST_ADDR, 'BINBOT')
        self.PRINT_FLAG = None
        self.POSE_FLAG = None
        if(os.path.isfile(self.MAP_DIR + 'map.pgm')):
            os.remove(self.MAP_DIR + 'map.pgm')
        if(os.path.isfile(self.MAP_DIR + 'map.yaml')):
            os.remove(self.MAP_DIR + 'map.yaml')
        self.MAP_FLAG = None
        # Subscriptions
        rospy.Subscriber('/bin_bot_base/state', String, self.BinStateCallback)
        rospy.Subscriber(
            '/amcl_pose', PoseWithCovarianceStamped, self.BinPoseCallback)
        # Publications
        self.POSE_TOPIC = rospy.Publisher('/bin_bot_base/goal', PoseStamped)
        self.SERV_STATE = rospy.Publisher('/client_node/serv_state', String)

    # State callback
    def BinStateCallback(self, DATA):
        self.BIN_CLIENT.SetState(DATA.data)

    # Pose callback (w/ conversion to euler)
    def BinPoseCallback(self, DATA):
        POSE_DICT = PoseCovarianceToDict(DATA)
        QUAT = [POSE_DICT['pose']['orientation']['x'],
                POSE_DICT['pose']['orientation']['y'],
                POSE_DICT['pose']['orientation']['z'],
                POSE_DICT['pose']['orientation']['w']]
        EUL = euler_from_quaternion(QUAT)
        POSE_DICT = QuaternionToEulerDict(POSE_DICT, EUL)
        self.BIN_CLIENT.SetPose(str(POSE_DICT))

    # Work callback, executed at RATE (defined at top)
    def WorkCallback(self):
        CUR_SERV_STATE = self.BIN_CLIENT.ServState()
        self.SERV_STATE.publish(CUR_SERV_STATE)
        # Display server state transitions in client console (debugging)
        if(self.PRINT_FLAG != CUR_SERV_STATE):
            self.PRINT_FLAG = CUR_SERV_STATE
            rospy.loginfo(self.PRINT_FLAG)

        if(CUR_SERV_STATE == 'BIN_TO_ARM'):
            if(not self.POSE_FLAG):
                POSE_STRING = self.BIN_CLIENT.RecvPose('OBJ')
                rospy.loginfo('POSE: ' + POSE_STRING)
                if(isinstance(POSE_STRING, str)):
                    self.POSE_FLAG = True
                    POSE_DICT = ast.literal_eval(POSE_STRING)
                    ROS_POSE = PoseStamped()
                    ROS_POSE = DictToPoseStamped(ROS_POSE, POSE_DICT)
                    self.POSE_TOPIC.publish(ROS_POSE)
        else:
            self.POSE_FLAG = None

        if(CUR_SERV_STATE == 'MAP_AT_SERVER'):
            if(not self.MAP_FLAG):
                self.BIN_CLIENT.RecvMap(self.MAP_DIR)
                self.MAP_FLAG = True
        else:
            self.MAP_FLAG = os.path.isfile(self.MAP_DIR + 'map.pgm') and \
                os.path.isfile(self.MAP_DIR + 'map.yaml')

    # Spin function to call work callback while ros running
    # and start/stop client synchronization
    def Spin(self):
        self.BIN_CLIENT.Start()
        rate = rospy.Rate(RATE)
        while not rospy.is_shutdown():
            self.WorkCallback()
            rate.sleep()
        self.BIN_CLIENT.Stop()

if __name__ == "__main__":
    rospy.init_node('bin_client_node')
    print 'Enter port: '
    PORT = int(raw_input(''))
    HOST_ADDR = ('192.168.0.117', PORT)
    MAP_DIR = '/home/sglvladi/map/'
    CLIENT_NODE = BinClientNode(HOST_ADDR, MAP_DIR)
    CLIENT_NODE.Spin()
