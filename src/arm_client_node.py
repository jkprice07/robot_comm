#!/usr/bin/python
#
#  Jason Price
#  Version: 2.1

import rospy
import time
import threading
import ast
import os
from read_settings import *
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from sensor_msgs.msg import Image
from clientclass import BotClient
from ros_image_conv import RosImageToPPMString
from pose_dict_tf import *

RATE = 2


class ArmClientNode:

    ######################################################
    #  Initialise client with server address HOST_ADDR and 
    #  map file directory MAP_DIR.  A client is initialised
    #  for the ARM and the BASE, and any exisiting map files
    #  are deleted on startup.
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
        #  Subscriptions for ARM state, BASE state, 
        #  BASE pose and ARM image data.
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
        #  Publications for server state and object pose goal.
        self.POSE_TOPIC = rospy.Publisher('/arm_bot_base/goal', PoseStamped)
        self.SERV_STATE = rospy.Publisher('/client_node/serv_state', String)

    #######################################
    #  Updates ARM state in client.
    def ArmStateCallback(self, DATA):
        self.ARM_CLIENT.SetState(DATA.data)

    #######################################
    #  Updates BASE state in client.
    def ArmBaseStateCallback(self, DATA):
        self.ARM_BASE_CLIENT.SetState(DATA.data)

    ###############################################
    #  Updates ARM pose in client.  Converts pose
    #  to a dictionary, converts the data to euler
    #  co-ords from quaternion and send to server.
    def ArmBasePoseCallback(self, DATA):
        POSE_DICT = PoseCovarianceToDict(DATA)
        POSE_DICT_STRING = str(QuaternionToEulerDict(POSE_DICT))
        self.ARM_BASE_CLIENT.SetPose(POSE_DICT_STRING)

    ######################################################
    #  When the server is waiting for an image file to be
    #  uploaded (for user decision), ROS geometry_msgs 
    #  Image  converted to PPM and sent to server.
    def ImageCallback(self, DATA):
        CUR_SERV_STATE = self.ARM_CLIENT.ServState()
        if(CUR_SERV_STATE == 'ARM_SEARCH'):
            if(not self.IMAGE_FLAG):
                PPM_DATA = RosImageToPPMString(DATA)
                self.ARM_CLIENT.SendFile(PPM_DATA, 'OBJ')
                self.IMAGE_FLAG = True
        elif(CUR_SERV_STATE == 'ARM_PICKUP'):
            if(not self.IMAGE_FLAG):
                PPM_DATA = RosImageToPPMString(DATA)
                self.ARM_CLIENT.SendFile(PPM_DATA, 'VER')
                self.IMAGE_FLAG = True
        else:
            self.IMAGE_FLAG = None

    ####################################################
    #  Work callback, executed at RATE (defined at top).
    def WorkCallback(self):
        #  Publish current server state.
        CUR_SERV_STATE = self.ARM_CLIENT.ServState()
        self.SERV_STATE.publish(CUR_SERV_STATE)
        #  Display current server state (if updated).
        if(self.PRINT_FLAG != CUR_SERV_STATE):
            self.PRINT_FLAG = CUR_SERV_STATE
            rospy.loginfo(self.PRINT_FLAG)
        #  Reset IMAGE_FLAG when in un-related state.
        if((CUR_SERV_STATE != 'ARM_SEARCH') or
                (CUR_SERV_STATE != 'ARM_PICKUP')):
            self.IMAGE_FLAG = None
        #  When object is found (server state FOUND_OBJ),
        #  an object pose is downloaded, converted to PoseStamped()
        #  message type and published to necessary ROS topic.
        if(CUR_SERV_STATE == 'FOUND_OBJ'):
            if(not self.POSE_FLAG):
                POSE_STRING = self.ARM_CLIENT.RecvPose('OBJ')
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
                self.ARM_CLIENT.RecvMap(self.MAP_DIR)
                self.MAP_FLAG = True
        else:
            self.MAP_FLAG = os.path.isfile(self.MAP_DIR + '/map.pgm') and \
                os.path.isfile(self.MAP_DIR + '/map.yaml')
                
    ####################################################
    #  Spin function starts/stops ARM and BASE clients 
    #  and executes Work Callback at desired RATE.
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
