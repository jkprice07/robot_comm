from tf.transformations import euler_from_quaternion, quaternion_from_euler
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped


#################################################### 
#  Converts ROS PoseStamped data type to dictionary.
def PoseStampedToDict(POSE_IN):
    POSE_DICT = {}
    POSE_DICT['header'] = {}
    POSE_DICT['header']['seq'] = POSE_IN.header.seq
    POSE_DICT['header']['stamp'] = {}
    POSE_DICT['header']['stamp']['secs'] = POSE_IN.header.stamp.secs
    POSE_DICT['header']['stamp']['nsecs'] = POSE_IN.header.stamp.nsecs
    POSE_DICT['header']['frame_id'] = 'map'
    POSE_DICT['pose'] = {}
    POSE_DICT['pose']['position'] = {}
    POSE_DICT['pose']['position']['x'] = POSE_IN.pose.position.x
    POSE_DICT['pose']['position']['y'] = POSE_IN.pose.position.y
    POSE_DICT['pose']['position']['z'] = POSE_IN.pose.position.z
    POSE_DICT['pose']['orientation'] = {}
    POSE_DICT['pose']['orientation']['x'] = POSE_IN.pose.orientation.x
    POSE_DICT['pose']['orientation']['y'] = POSE_IN.pose.orientation.y
    POSE_DICT['pose']['orientation']['z'] = POSE_IN.pose.orientation.z
    POSE_DICT['pose']['orientation']['w'] = POSE_IN.pose.orientation.w
    return POSE_DICT

###################################################################
#  Converts ROS PoseStampedWithCovariance data type to dictionary.
def PoseCovarianceToDict(POSE_IN):
    POSE_DICT = {}
    POSE_DICT['header'] = {}
    POSE_DICT['header']['seq'] = POSE_IN.header.seq
    POSE_DICT['header']['stamp'] = {}
    POSE_DICT['header']['stamp']['secs'] = POSE_IN.header.stamp.secs
    POSE_DICT['header']['stamp']['nsecs'] = POSE_IN.header.stamp.nsecs
    POSE_DICT['header']['frame_id'] = 'map'
    POSE_DICT['pose'] = {}
    POSE_DICT['pose']['position'] = {}
    POSE_DICT['pose']['position']['x'] = POSE_IN.pose.pose.position.x
    POSE_DICT['pose']['position']['y'] = POSE_IN.pose.pose.position.y
    POSE_DICT['pose']['position']['z'] = POSE_IN.pose.pose.position.z
    POSE_DICT['pose']['orientation'] = {}
    POSE_DICT['pose']['orientation']['x'] = POSE_IN.pose.pose.orientation.x
    POSE_DICT['pose']['orientation']['y'] = POSE_IN.pose.pose.orientation.y
    POSE_DICT['pose']['orientation']['z'] = POSE_IN.pose.pose.orientation.z
    POSE_DICT['pose']['orientation']['w'] = POSE_IN.pose.pose.orientation.w
    return POSE_DICT

#################################################### 
#  Converts python dictionary to ROS PoseStamped.
def DictToPoseStamped(DICT_IN):
    POSE = PoseStamped()
    POSE.header.frame_id = DICT_IN['header']['frame_id']
    POSE.pose.position.x = DICT_IN['pose']['position']['x']
    POSE.pose.position.y = DICT_IN['pose']['position']['y']
    POSE.pose.position.z = DICT_IN['pose']['position']['z']
    POSE.pose.orientation.x = DICT_IN['pose']['orientation']['x']
    POSE.pose.orientation.y = DICT_IN['pose']['orientation']['y']
    POSE.pose.orientation.z = DICT_IN['pose']['orientation']['z']
    POSE.pose.orientation.w = DICT_IN['pose']['orientation']['w']
    return POSE

######################################################
#  Converts quaternion dictionary to euler dictionary.
def QuaternionToEulerDict(DICT_IN):
    QUAT = [DICT_IN['pose']['orientation']['x'],
            DICT_IN['pose']['orientation']['y'],
            DICT_IN['pose']['orientation']['z'],
            DICT_IN['pose']['orientation']['w']]
    EUL = euler_from_quaternion(QUAT)
    TF_DICT = {}
    TF_DICT['pose'] = {}
    TF_DICT['pose']['orientation'] = {}
    TF_DICT['pose']['orientation']['roll'] = EUL[0]
    TF_DICT['pose']['orientation']['pitch'] = EUL[1]
    TF_DICT['pose']['orientation']['yaw'] = EUL[2]
    DICT_IN['pose']['orientation'] = TF_DICT['pose']['orientation']
    return DICT_IN

######################################################
#  Converts euler dictionary to quaternion dictionary.
def EulerToQuaternionDict(DICT_IN):
    EUL = DICT_IN['pose']['orientation']['yaw'],\
        DICT_IN['pose']['orientation']['pitch'],\
        DICT_IN['pose']['orientation']['roll']
    QUAT = quaternion_from_euler(EUL)
    TF_DICT = {}
    TF_DICT['pose'] = {}
    TF_DICT['pose']['orientation'] = {}
    TF_DICT['pose']['orientation']['x'] = QUAT[0]
    TF_DICT['pose']['orientation']['y'] = QUAT[1]
    TF_DICT['pose']['orientation']['z'] = QUAT[2]
    TF_DICT['pose']['orientation']['w'] = QUAT[3]
    DICT_IN['pose']['orientation'] = TF_DICT['pose']['orientation']
    return DICT_IN
