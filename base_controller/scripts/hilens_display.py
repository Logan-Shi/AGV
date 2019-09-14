#!/usr/bin/env python
# -*- coding:utf-8 -*-
from socket import *
from time import ctime
import time
import cv2
import numpy
import sys
import os
import rospy
from std_msgs.msg import UInt8

def cvShowImage(imgName,img):
    cv2.imshow(imgName,img)
    cv2.waitKey(0)
    cv2.destroyAllWindows()

def recvall(sock, count):
    buf = b''
    while count:
        newbuf = sock.recv(count)
        if not newbuf: return None
        buf += newbuf
        count -= len(newbuf)
    return buf

def getPhoto(hilens_socket):
    '''从hilens中获得一张当前摄像头的照片
    
    Arguments:
        hilens_socket {socket.socket} -- [用于与hilens连接的socket]
    
    Returns:
        [numpy.ndarray ] -- [hilens当前捕捉到的图片，色彩空间为BGR]
    '''
    hilens_socket.send('photo'.encode())
    length = recvall(hilens_socket,16)
    stringData = recvall(hilens_socket, int(length))
    data = numpy.frombuffer(stringData, numpy.uint8)
    decimg=cv2.imdecode(data,cv2.IMREAD_COLOR)
    hilens_socket.send('done'.encode())
    return decimg

def makeDetection(hilens_socket,keep_top_k):
    '''从hilens中获得当前检测到的keep_top_k个对象的标签和置信度
    
    [description]
    
    Arguments:
        hilens_socket {socket.socket} -- [用于与hilens连接的socket]
        keep_top_k {int} -- [让hilens返回的检测到的对象的个数]
    
    Returns:
        [list] -- [一个包含keep_top_k个元组的列表，其中每个元组为(标签,置信度)]
    '''
    bufsize = 1024
    getResult = False
    while not getResult:
        try:
            hilens_socket.send('detection'.encode())
            receive = hilens_socket.recv(bufsize)
            if not len(receive):
                print("no confirm receive")
            else:
                print('receive confirm')
            hilens_socket.send(str(keep_top_k).encode())
            receive = hilens_socket.recv(bufsize)
            if not len(receive):
                print("no rkeep_top_k receive")
                continue
            rkeep_top_k = int(receive.decode())
            #print("get rkeep_top_k: ",rkeep_top_k)
            hilens_socket.send('done'.encode())
            result = []
            for i in range(rkeep_top_k):
                receive = (hilens_socket.recv(bufsize)).decode()
                message = receive.split(',')
                clas = int(message[0])
                conf = float(message[1])
                result.append((clas,conf))
                if(i != rkeep_top_k - 1):
                    hilens_socket.send('done'.encode())
            hilens_socket.send('done'.encode())
            getResult = True
            time.sleep(0.01)
        except:
            print(sys.exc_info()[0])
            print(sys.exc_info()[1])
            print("making detection again")
            continue
    #print("detection result:",result)
    return result
        

def connectToHilens(hilens_socket,serverName,serverPort):
    connect = False
    while not connect:
        try:
            hilens_socket.connect((serverName,serverPort))
        except:
            print(sys.exc_info()[0])
            print(sys.exc_info()[1])
            print('trying connecting')
            continue
        connect = True
        print('connected')     
    return True


class hilens():
    def __init__(self, arg):
        self.arg = arg
        rospy.init_node('hilens', anonymous=True)
        self.hilensData = UInt8()
        self.dataPub = rospy.Publisher('hilensData',UInt8, queue_size=1)
        self.rate = rospy.Rate(10)
        rospy.on_shutdown(self._shutdown)
        self.hilens_socket = socket(AF_INET,SOCK_STREAM)

    def spin(self):
        while not rospy.is_shutdown():
            self.main('192.168.2.111',8080)
            self.dataPub.publish(self.hilensData)
            self.rate.sleep()

    def _shutdown(self):
        try:
            rospy.loginfo('shutting down')
            self.hilens_socket.close()
        except:
            print(sys.exc_info()[0])

    def main(self,serverName,serverPort):
        bufsize = 1024
        self.hilens_socket = socket(AF_INET,SOCK_STREAM)
        connectToHilens(self.hilens_socket,serverName,serverPort)
        try:
            # img = getPhoto(self.hilens_socket)
            # cvShowImage('imgFromHiles',img)
            # sys.sleep(0.02)
            # self.hilensData = 1
            data = makeDetection(self.hilens_socket,4)
            if data[0][0] == 1:
                if data[0][1] < 0.3:
                    self.hilensData = 0
            elif data[0][1] < 0.1:
                self.hilensData = 0
            else:
                self.hilensData = data[0][0]
            self.hilens_socket.close()
        except:
            try:
                self.hilens_socket.close()
            except:
                print(sys.exc_info()[0])


if __name__=="__main__":
    try:
        hilens = hilens(1)
        hilens.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("hilens stopped.")
