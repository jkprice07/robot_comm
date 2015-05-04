#!/usr/bin/python

# Version: 2.0

import rospy
import time
import threading
import ast
import os
from read_settings import *
from tf.transformations import euler_from_quaternion
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from sensor_msgs.msg import Image
from clientclass import BotClient
from ros_image_conv import RosImageToPPMString
from pose_dict_tf import *

RATE = 10


class ArmClientNode:

    def __init__(self, HOST_ADDR, MAP_DIR):
        self.HOST_ADDR = HOST_ADDR
        self.MAP_DIR = MAP_DIR
        self.ARM_CLIENT = BotClient(HOST_ADDR, 'ARMBOT')
        self.ARM_BASE_CLIENT = BotClient(HOST_ADDR, 'BASEBOT')
        self.IMAGE_FLAG = None
        self.PRINT_FLAG = None
        self.POSE_FLAG = None
        if(os.path.isfile(self.MAP_DIR + '/map.pgm')):
            os.remove(self.MAP_DIR + '/map.pgm')
        if(os.path.isfile(self.MAP_DIR + '/map.yaml')):
            os.remove(self.MAP_DIR + '/map.yaml')
        self.MAP_FLAG = None
        # Subscriptions
        rospy.Subscriber('/uarm/state',
                         String,
                         self.ArmStateCallback)
        rospy.Subscriber('/arm_bot_base/state',
                         String,
                         self.ArmBaseStateCallback)
        rospy.Subscriber('/amcl_pose',
                         PoseWithCovarianceStamped,
                         self.ArmBasePoseCallback)
        rospy.Subscriber('/usb_cam/image_raw',
                         Image,
                         self.ImageCallback)
        # Publications
        self.POSE_TOPIC = rospy.Publisher('/arm_bot_base/goal', PoseStamped)
        self.SERV_STATE = rospy.Publisher('/client_node/serv_state', String)

    def ArmStateCallback(self, DATA):
        self.ARM_CLIENT.SetState(DATA.data)

    def ArmBaseStateCallback(self, DATA):
        self.ARM_BASE_CLIENT.SetState(DATA.data)

    def ArmBasePoseCallback(self, DATA):
        POSE_DICT = PoseCovarianceToDict(DATA)
        QUAT = [POSE_DICT['pose']['orientation']['x'],
                POSE_DICT['pose']['orientation']['y'],
                POSE_DICT['pose']['orientation']['z'],
                POSE_DICT['pose']['orientation']['w']]
        EUL = euler_from_quaternion(QUAT)
        POSE_DICT = QuaternionToEulerDict(POSE_DICT, EUL)
        self.ARM_BASE_CLIENT.SetPose(str(POSE_DICT))

    def ImageCallback(self, DATA):
        if(self.ARM_CLIENT.ServState() == 'ARM_SEARCH'):
            if(not self.IMAGE_FLAG):
                PPM_DATA = RosImageToPPMString(DATA)
                self.ARM_CLIENT.SendFile(PPM_DATA, 'OBJ')
                self.IMAGE_FLAG = True
        elif(self.ARM_CLIENT.ServState() == 'ARM_PICKUP'):
            if(not self.IMAGE_FLAG):
                PPM_DATA = RosImageToPPMString(DATA)
                self.ARM_CLIENT.SendFile(PPM_DATA, 'VER')
                self.IMAGE_FLAG = True

    def WorkCallback(self):
        CUR_SERV_STATE = self.ARM_CLIENT.ServState()
        self.SERV_STATE.publish(CUR_SERV_STATE)

        if(self.PRINT_FLAG != CUR_SERV_STATE):
            self.PRINT_FLAG = CUR_SERV_STATE
            rospy.loginfo(self.PRINT_FLAG)

        if((CUR_SERV_STATE != 'ARM_SEARCH') or
                (CUR_SERV_STATE != 'ARM_PICKUP')):
            self.IMAGE_FLAG = None

        if(CUR_SERV_STATE == 'FOUND_OBJ'):
            if(not self.POSE_FLAG):
                POSE_STRING = self.ARM_CLIENT.RecvPose('OBJ')
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
                self.ARM_CLIENT.RecvMap(self.MAP_DIR)
                self.MAP_FLAG = True
        else:
            self.MAP_FLAG = os.path.isfile(self.MAP_DIR + '/map.pgm') and \
                os.path.isfile(self.MAP_DIR + '/map.yaml')

    def Spin(self):
        self.ARM_CLIENT.Start()
        self.ARM_BASE_CLIENT.Start()
        rate = rospy.Rate(RATE)
        while not rospy.is_shutdown():
            self.WorkCallback()
            rate.sleep()
        self.ARM_CLIENT.Stop()
        self.ARM_BASE_CLIENT.Stop()


if __name__ == "__main__":
    rospy.init_node('arm_client_node')
    IP, MAP_DIR = ReadSettings('ARMBOT')
    PORT = int(raw_input('Enter port:\n'))
    HOST_ADDR = (IP, PORT)
    CLIENT_NODE = ArmClientNode(HOST_ADDR, MAP_DIR)
    CLIENT_NODE.Spin()
