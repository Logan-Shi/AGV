#!/usr/bin/env python


import rospy
import cv2
from geometry_msgs.msg import Point
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import numpy as np


class HuePicker():
    
    def __init__(self):
        self.pt = None
        self.pts = []
        self.image = None

        self.bridge = CvBridge()
        self.sub_pt = rospy.Subscriber(\
                "/zed/rgb/image_rect_color_mouse_rgb", Point, self.cb_pt)
        self.sub_image = rospy.Subscriber(\
                "/zed/rgb/image_rect_color", Image, self.cb_img)
        
    def cb_img(self, msg):
        self.image = msg
    
    def cb_pt(self, msg):
        self.pt = int(msg.y), int(msg.x)
        # print self.pt
        if self.image is None:
            print "NO IMAGE"
            return

        img  = self.bridge.imgmsg_to_cv2(self.image)            
        hsv_img = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        vals = hsv_img[self.pt]
        self.pts.append(vals)

        pts = np.array(self.pts)
        min_thresh = np.min(pts, axis=0).tolist()
        max_thresh = np.max(pts, axis=0).tolist()
        print "thresholds: %s, %s " %(min_thresh, max_thresh)

# rospy.init_node("huepicker")
print """ 
open rqt_image_view and subscribe to /zed/rgb/image_rect_color. 
Use the check box to select /zed/rgb/image_rect_color_mouse_rgb 
to publish mouse clicks.

Put the cone in view of the camera and click on the cone multiple times. 

This code will find the min max values of the pixels from the mouse click converted to HSV values.

Use this code whenever object detector isn't working or to
detect new color range
"""

if __name__=="__main__":
    # initalize the ros node
    rospy.init_node('hue_picker_node')
    
    # create the object detector
    node = HuePicker()
    
    # continue running node until it is killed from outside process
    rospy.spin()
