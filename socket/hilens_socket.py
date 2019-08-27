#import hilens
from socket import *

HOST = ''
PORT = 8088
bufsize = 1024


socket_tx2 = socket(AF_INET,SOCK_STREAM)
socket_tx2.bind((HOST,PORT))
socket_tx2.listen(5)

while True:
    connection,address = socket_tx2.accept()
    while True:
        try:
            flag = connection.recv(bufsize)
        except IOError:
            connection.close()
            break
        if not flag:
            break
        print(flag)
        returnData = flag
        connection.send(returnData)
    connection.close()
socket_tx2.close()



