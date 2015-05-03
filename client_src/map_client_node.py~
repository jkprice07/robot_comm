#!/usr/bin/python

# Version: 2.0

import rospy
import subprocess
import os
from tf.transformations import euler_from_quaternion
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped
from clientclass import BotClient
from pose_dict_tf import *

RATE = 2


class MapClientNode:

    def __init__(self, HOST_ADDR, MAP_DIR):
        self.HOST_ADDR = HOST_ADDR
        self.MAP_DIR = MAP_DIR
        self.MAP_CLIENT = BotClient(HOST_ADDR, 'MAPBOT')
        self.PRINT_FLAG = None
	if(os.path.isfile(self.MAP_DIR + 'map.pgm')):
            os.remove(self.MAP_DIR + 'map.pgm')
        if(os.path.isfile(self.MAP_DIR + 'map.yaml')):
            os.remove(self.MAP_DIR + 'map.yaml')
        self.MAP_FLAG = None
        # Subscriptions
        rospy.Subscriber('/map_bot_base/state', String, self.MapStateCallback)
        rospy.Subscriber('/slam_out_pose', PoseStamped, self.MapPoseCallback)
        # Publications
        self.SERV_STATE = rospy.Publisher('/client_node/serv_state', String)

    def MapStateCallback(self, DATA):
        self.MAP_CLIENT.SetState(DATA.data)

    def MapPoseCallback(self, DATA):
        POSE_DICT = PoseStampedToDict(DATA)
        QUAT = [POSE_DICT['pose']['orientation']['x'],
                POSE_DICT['pose']['orientation']['y'],
                POSE_DICT['pose']['orientation']['z'],
                POSE_DICT['pose']['orientation']['w']]
        EUL = euler_from_quaternion(QUAT)
        POSE_DICT = QuaternionToEulerDict(POSE_DICT, EUL)
        self.MAP_CLIENT.SetPose(str(POSE_DICT))

    def WorkCallback(self):
        CUR_SERV_STATE = self.MAP_CLIENT.ServState()
        self.SERV_STATE.publish(CUR_SERV_STATE)

        if(self.PRINT_FLAG != CUR_SERV_STATE):
            self.PRINT_FLAG = CUR_SERV_STATE
            rospy.loginfo(self.PRINT_FLAG)

        if(CUR_SERV_STATE == 'MAP_DONE'):
            if(not self.MAP_FLAG):
                subprocess.call('rosrun map_server map_saver -f map',  shell=True)
                FILE = open('/home/map.yaml', 'rb')
                DATA = FILE.read()
                FILE.close()
                self.MAP_CLIENT.SendFile(DATA, 'MAP_YAML')
                FILE = open('/home/map.pgm', 'rb')
                DATA = DATA + FILE.read()
                FILE.close()
                self.MAP_CLIENT.SendFile(DATA, 'MAP_PGM')
                self.MAP_FLAG = True
        else:
            self.MAP_FLAG = os.path.isfile(
            self.MAP_DIR + 'map.pgm') and \
            	os.path.isfile(self.MAP_DIR + 'map.yaml')

    def Spin(self):
        self.MAP_CLIENT.Start()
        rate = rospy.Rate(RATE)
        while not rospy.is_shutdown():
            self.WorkCallback()
            rate.sleep()
        self.MAP_CLIENT.Stop()

if __name__ == "__main__":
    rospy.init_node('map_client_node')
    print 'Enter port: '
    PORT = int(raw_input(''))
    HOST_ADDR = ('192.168.0.117', PORT)
    MAP_DIR = '/home/smartlab-tb01/map/'
    CLIENT_NODE = MapClientNode(HOST_ADDR, MAP_DIR)
    CLIENT_NODE.Spin()
