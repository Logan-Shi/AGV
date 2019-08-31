from socket import *
from time import ctime

serverName = '192.168.2.111'
serverPort = 8088
bufsize = 1024

hilens_socket = socket(AF_INET,SOCK_STREAM)

connect = false

while not connect:
    try:
        hilens_socket.connect((serverName,serverPort))
    except:
        continue
    connect = True

while True:
    data = ctime()
    if not data:
        break
    hilens_socket.send(data.encode('utf-8'))
    returnData = hilens_socket.recv(bufsize)
    if not returnData:
        break
    print('Return time is:%s' %returnData.decode('utf-8'))
hilens_socket.close()
