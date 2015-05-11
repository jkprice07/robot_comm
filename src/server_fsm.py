from time import time


class FSM(object):

    def __init__(self):
        self.states = {}
        self.transitions = {}
        self.curState = None
        self.prevState = None
        self.trans = None

    def AddTransition(self, transName, transition):
        self.transitions[transName] = transition

    def AddState(self, stateName, state):
        self.states[stateName] = state

    def SetState(self, stateName):
        self.prevState = self.curState
        self.curState = self.states[stateName]

    def ToTransition(self, toTrans):
        self.trans = self.transitions[toTrans]

    def Execute(self):
        if(self.trans):
            self.trans.Execute()
            self.SetState(self.trans.toState)
            self.trans = None
        self.curState.Execute()


class State(object):

    def __init__(self, SERV):
        self.SERV = SERV

    def Execute(self):
        pass


class Transition(object):

    def __init__(self, toState):
        self.toState = toState

    def Execute(self):
        print 'Transitioning..'


class IDLE(State):

    def __init__(self, SERV):
        super(IDLE, self).__init__(SERV)

    def Execute(self):
        START = self.SERV.StartProcess()
        MAP_FLAG = self.SERV.MapFlag()
        if(START):
            if(MAP_FLAG):
                self.SERV.FSM.ToTransition('To_MAP_AT_SERVER')
            else:
                self.SERV.FSM.ToTransition('To_START')

    def StateName(self):
        return 'IDLE'


class START(State):

    def __init__(self, SERV):
        super(START, self).__init__(SERV)

    def Execute(self):
        BOT_DATA = self.SERV.BotData()
        if(BOT_DATA['STATES']['MAPBOT'] == 'FSM_AUTONOMY'):
            self.SERV.FSM.ToTransition('To_MAPPING_AUTO')

    def StateName(self):
        return 'START'


class MAPPING_AUTO(State):

    def __init__(self, SERV):
        super(MAPPING_AUTO, self).__init__(SERV)

    def Execute(self):
        BOT_DATA = self.SERV.BotData()
        if(BOT_DATA['STATES']['MAPBOT'] == 'FSM_MANUAL'):
            self.SERV.FSM.ToTransition('To_MAPPING_MAN')

    def StateName(self):
        return 'MAPPING_AUTO'


class MAPPING_MAN(State):

    def __init__(self, SERV):
        super(MAPPING_MAN, self).__init__(SERV)

    def Execute(self):
        USER_INPUT = self.SERV.UserData()
        if(USER_INPUT):
            self.SERV.FSM.ToTransition('To_MAP_DONE')

    def StateName(self):
        return 'MAPPING_MAN'


class MAP_DONE(State):

    def __init__(self, SERV):
        super(MAP_DONE, self).__init__(SERV)

    def Execute(self):
        MAP_FLAG = self.SERV.MapFlag()
        if(MAP_FLAG):
            self.SERV.FSM.ToTransition('To_MAP_AT_SERVER')

    def StateName(self):
        return 'MAP_DONE'


class MAP_AT_SERVER(State):

    def __init__(self, SERV):
        super(MAP_AT_SERVER, self).__init__(SERV)

    def Execute(self):
        BOT_DATA = self.SERV.BotData()
        if(BOT_DATA['STATES']['BASEBOT'] == 'FSM_IDLE'):
            if(BOT_DATA['STATES']['BINBOT'] == 'FSM_IDLE'):
                self.SERV.FSM.ToTransition('To_FINDING_OBJ')

    def StateName(self):
        return 'MAP_AT_SERVER'


class FINDING_OBJ(State):

    def __init__(self, SERV):
        super(FINDING_OBJ, self).__init__(SERV)

    def Execute(self):
        OBJ_POSE = self.SERV.ObjPose()
        if(OBJ_POSE):
            self.SERV.FSM.ToTransition('To_FOUND_OBJ')

    def StateName(self):
        return 'FINDING_OBJ'


class FOUND_OBJ(State):

    def __init__(self, SERV):
        super(FOUND_OBJ, self).__init__(SERV)

    def Execute(self):
        BOT_DATA = self.SERV.BotData()
        if(BOT_DATA['STATES']['BASEBOT'] == 'FSM_NAVIGATE'):
            self.SERV.FSM.ToTransition('To_ARM_TO_OBJ')

    def StateName(self):
        return 'FOUND_OBJ'


class ARM_TO_OBJ(State):

    def __init__(self, SERV):
        super(ARM_TO_OBJ, self).__init__(SERV)

    def Execute(self):
        BOT_DATA = self.SERV.BotData()
        if(BOT_DATA['STATES']['BASEBOT'] == 'FSM_WAIT_FOR_ACTION'):
            if(BOT_DATA['STATES']['ARMBOT'] == 'WAIT'):
                self.SERV.FSM.ToTransition('To_ARM_SEARCH')

    def StateName(self):
        return 'ARM_TO_OBJ'


class ARM_SEARCH(State):

    def __init__(self, SERV):
        super(ARM_SEARCH, self).__init__(SERV)

    def Execute(self):
        IMAGE_FLAG = self.SERV.ImageFlag()
        if(IMAGE_FLAG):
            self.SERV.FSM.ToTransition('To_USER_DEC')

    def StateName(self):
        return 'ARM_SEARCH'


# class USER_DEC(State):

#     def __init__(self, SERV):
#         super(USER_DEC, self).__init__(SERV)

#     def Execute(self):
#         USER_INPUT = self.SERV.UserData()
#         OBJ_POSE = self.SERV.ObjPose()
#         if(USER_INPUT == 'PICKUP'):
#             self.SERV.FSM.ToTransition('To_BIN_TO_ARM')
#         elif(USER_INPUT == 'LEAVE'):
#             self.SERV.PopObjPose()
#             if(OBJ_POSE):
#                 self.SERV.FSM.ToTransition('To_FOUND_OBJ')
#             else:
#                 self.SERV.SetCount()
#                 self.SERV.FSM.ToTransition('To_RESET')

#     def StateName(self):
#         return 'USER_DEC'

class USER_DEC(State):

    def __init__(self, SERV):
        super(USER_DEC, self).__init__(SERV)

    def Execute(self):
        self.SERV.FSM.ToTransition('To_BIN_TO_ARM')

    def StateName(self):
        return 'USER_DEC'


class BIN_TO_ARM(State):

    def __init__(self, SERV):
        super(BIN_TO_ARM, self).__init__(SERV)

    def Execute(self):
        BOT_DATA = self.SERV.BotData()
        if(BOT_DATA['STATES']['BINBOT'] == 'FSM_WAIT_FOR_ACTION'):
            self.SERV.FSM.ToTransition('To_BIN_AT_ARM')

    def StateName(self):
        return 'BIN_TO_ARM'


class BIN_AT_ARM(State):

    def __init__(self, SERV):
        super(BIN_AT_ARM, self).__init__(SERV)

    def Execute(self):
        BOT_DATA = self.SERV.BotData()
        if(BOT_DATA['STATES']['ARMBOT'] == 'VERIFY'):
            self.SERV.FSM.ToTransition('To_ARM_PICKUP')

    def StateName(self):
        return 'BIN_AT_ARM'


class ARM_PICKUP(State):

    def __init__(self, SERV):
        super(ARM_PICKUP, self).__init__(SERV)

    def Execute(self):
        IMAGE_FLAG = self.SERV.ImageFlag()
        if(IMAGE_FLAG):
            self.SERV.FSM.ToTransition('To_PICKUP_CHECK')

    def StateName(self):
        return 'ARM_PICKUP'


class PICKUP_CHECK(State):

    def __init__(self, SERV):
        super(PICKUP_CHECK, self).__init__(SERV)

    def Execute(self):
        self.SERV.FSM.ToTransition('To_ARM_DROPPING')

    def StateName(self):
        return 'PICKUP_CHECK'

# class PICKUP_CHECK(State):

#     def __init__(self, SERV):
#         super(PICKUP_CHECK, self).__init__(SERV)

#     def Execute(self):
#         USER_INPUT = self.SERV.UserData()
#         if(USER_INPUT == 'SUCC'):
#             self.SERV.FSM.ToTransition('To_ARM_DROPPING')
#         if(USER_INPUT == 'FAIL'):
#             self.SERV.FSM.ToTransition('To_BIN_AT_ARM')

#     def StateName(self):
#         return 'PICKUP_CHECK'


class ARM_DROPPING(State):

    def __init__(self, SERV):
        super(ARM_DROPPING, self).__init__(SERV)

    def Execute(self):
        BOT_DATA = self.SERV.BotData()
        if(BOT_DATA['STATES']['ARMBOT'] == 'DROPPED'):
            self.SERV.PopObjPose()
            self.SERV.SetCount()
            self.SERV.FSM.ToTransition('To_BIN_TO_BASE')

    def StateName(self):
        return 'ARM_DROPPING'


class BIN_TO_BASE(State):

    def __init__(self, SERV):
        super(BIN_TO_BASE, self).__init__(SERV)

    def Execute(self):
        BOT_DATA = self.SERV.BotData()
        OBJ_POSE = self.SERV.ObjPose()
        if(BOT_DATA['STATES']['BINBOT'] == 'FSM_MOVE_TO_BASE'):
            if(OBJ_POSE):
                if((self.SERV.ViewCount() + 20) < time()):
                    self.SERV.FSM.ToTransition('To_FOUND_OBJ')
            else:
                self.SERV.SetCount()
                self.SERV.FSM.ToTransition('To_RESET')

    def StateName(self):
        return 'BIN_TO_BASE'


class RESET(State):

    def __init__(self, SERV):
        super(RESET, self).__init__(SERV)

    def Execute(self):
        if((self.SERV.ViewCount() + 5) < time()):
            self.SERV.FSM.ToTransition('To_IDLE')

    def StateName(self):
        return 'RESET'
