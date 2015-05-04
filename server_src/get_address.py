import os


def GetAddress():
    PORT = int(raw_input('Enter port:\n'))
    FILE = open(os.getcwd() + '/host_address.txt', 'rb')
    IP = FILE.read()
    if(IP[len(IP) - 1] == '\n'):
        IP = IP[0:len(IP) - 1]
    FILE.close()
    HOST_ADDR = (IP, PORT)
    return HOST_ADDR
