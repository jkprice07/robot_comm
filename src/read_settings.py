import os


def Decode(LINE):
    LINE_DATA = LINE[9:len(LINE) - 1]
    return LINE_DATA


def ReadSettings(TYPE):
    #FILE = open()
    if(os.path.isfile(os.getcwd() + '/settings.txt')):
        FILE = open(os.getcwd() + '/settings.txt', 'rb')
    elif(os.path.isfile(os.getcwd() + '/src/settings.txt')):
        FILE = open(os.getcwd() + '/src/settings.txt', 'rb')
    elif(os.path.isfile(os.path.split(os.path.abspath(os.getcwd()))[0] + '/settings.txt')):
        FILE = open(os.path.split(os.path.abspath(os.getcwd()))[0] + '/settings.txt', 'rb')
    else:
        return 0
    IP = Decode(FILE.readline())
    SERV_MAP = Decode(FILE.readline())
    SERV_IM = Decode(FILE.readline())
    ARM_MAP = Decode(FILE.readline())
    BIN_MAP = Decode(FILE.readline())
    MAP_MAP = Decode(FILE.readline())
    FLY_MAP = Decode(FILE.readline())
    FILE.close()
    if(TYPE == 'SERVER'):
        return IP, SERV_MAP, SERV_IM
    elif(TYPE == 'ARMBOT'):
        return IP, ARM_MAP
    elif(TYPE == 'BINBOT'):
        return IP, BIN_MAP
    elif(TYPE == 'MAPBOT'):
        return IP, MAP_MAP
    elif(TYPE == 'FLYBOT'):
        return IP, FLY_MAP
