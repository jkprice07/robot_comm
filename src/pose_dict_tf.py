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


def DictToPoseStamped(POSE, DICT_IN):
    POSE_ACT = POSE
    POSE_ACT.header.frame_id = DICT_IN['header']['frame_id']
    POSE_ACT.pose.position.x = DICT_IN['pose']['position']['x']
    POSE_ACT.pose.position.y = DICT_IN['pose']['position']['y']
    POSE_ACT.pose.position.z = DICT_IN['pose']['position']['z']
    POSE_ACT.pose.orientation.x = DICT_IN['pose']['orientation']['x']
    POSE_ACT.pose.orientation.y = DICT_IN['pose']['orientation']['y']
    POSE_ACT.pose.orientation.z = DICT_IN['pose']['orientation']['z']
    POSE_ACT.pose.orientation.w = DICT_IN['pose']['orientation']['w']
    return POSE_ACT


def QuaternionToEulerDict(DICT_IN, EUL):
    TF_DICT = {}
    TF_DICT['pose'] = {}
    TF_DICT['pose']['orientation'] = {}
    TF_DICT['pose']['orientation']['roll'] = EUL[0]
    TF_DICT['pose']['orientation']['pitch'] = EUL[1]
    TF_DICT['pose']['orientation']['yaw'] = EUL[2]
    DICT_IN['pose']['orientation'] = TF_DICT['pose']['orientation']
    return DICT_IN


def EulerToQuaternionDict(DICT_IN, QUAT):
    TF_DICT = {}
    TF_DICT['pose'] = {}
    TF_DICT['pose']['orientation'] = {}
    TF_DICT['pose']['orientation']['x'] = QUAT[0]
    TF_DICT['pose']['orientation']['y'] = QUAT[1]
    TF_DICT['pose']['orientation']['z'] = QUAT[2]
    TF_DICT['pose']['orientation']['w'] = QUAT[3]
    DICT_IN['pose']['orientation'] = TF_DICT['pose']['orientation']
    return DICT_IN
