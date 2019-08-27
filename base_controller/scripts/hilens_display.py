#!/usr/bin/env python
import rospy
from flask import Flask
from flask import request
from geometry_msgs.msg import Point
import numpy as np
conf_threshold = 1
size_threshold = 100
app = Flask(__name__)
@app.route("/follow_detection_result", methods=['POST', 'GET'])
def webhook():
    data = request.values.to_dict()
    bbox = data['bbox']
    for i in range(len(bbox)):
        if(bbox[i] == ']'):
            break
    pos = bbox[4:i]
    position = []
    cur_pos = ''
    last = ' '
    cur = ' '
    for j in range(len(pos)):
        cur = pos[j]
        if((last == ' ') and (cur == ' ')):
            pass
        elif((cur != ' ')):
            cur_pos = cur_pos + cur
        elif((last != ' ') and (cur == ' ')):
            position.append(cur_pos)
            cur_pos = ''
        last = cur
        position.append(cur_pos)
    _left, _up, _right, _bottom = position
    left = float(_left)
    up = float(_up)
    right = float(_right)
    bottom = float(_bottom)
    w = right - left
    h = bottom - up
    size = w * h
    center = [(left + right) // 2, (up + bottom) // 2]
    confidence = data['confidence']
    for i in range(len(confidence)):
        if(confidence[i] == ']'):
            break
    _conf = confidence[3:i]
    conf = float(_conf)
    objclass = data['class']
    for i in range(len(objclass)):
        if(objclass[i] == '.'):
            break
    _objcls = objclass[3:i]
    objcls = int(_objcls)
    hilensDetMsg = Point()
    print(objcls)
    print(size)
    
    '''
    if conf > conf_threshold and size > size_threshold:
        hilensDetMsg.has_object = True
        hilensDetMsg.object_class = objcls
        hilensDetMsg.object_size = size
        hilensDetMsg.object_center.x = center[0]
        hilensDetMsg.object_center.y = center[1]
'''
    hilensDetMsg.x = center[0]
    hilensDetMsg.y = center[1]
    hilensDetPub.publish(hilensDetMsg)
    
if __name__ == "__main__":
    rospy.init_node('hilens', anonymous=True)
    hilensDetPub = rospy.Publisher('hilens_detection', Point, queue_size=1)
    app.run(host="198.168.55.1", port="8088")
