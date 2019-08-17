#!/usr/bin/env python
"""
This program receives an image from a rostopic and 
converts it to a cv2 image. Then it uses the cv2 functions
to detect a blob of a desired color. It draws a bounding
box over the blob and publishes this modified image
to the ~detection_image topic.
"""
DEBUG = True # publish images
import cv2
import numpy as np
import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import Float32
from cv_bridge import CvBridge, CvBridgeError
import sys
from ta_lab4.msg import Detection
#from threading import Thread
#from ackermann_msgs.msg import AckermannDriveStamped, AckermannDrive

"""
The detector class opereates only on opencv images.  It
will detect Yellow blobs in an image and mark them with 
bounding boxes. It will label MAX_DETECTIONS number of 
blobs, where the blobs are sorted by contour area.
"""

class Detector:
    # hsv ranges for yellow detections
    # COLOR = [np.array(x, np.uint8) for x in [\
    #      [  9 ,189 ,150], [ 15, 255 ,253]        
    #     ] ]
    COLOR = [np.array(x, np.uint8) for x in [\
         [  6 ,249 ,214], [ 11, 255 ,253]        
        ] ]
    # number of blobs to detect
    MAX_DETECTIONS = 1

    # desired area of cone detection
    DESIRED_AREA = 20000 # pixels
    
    """ 
    get_filtered_contours will segment images based on color and then
    return the largest contours. 
    """
    def get_filtered_contours(self,img):
        # convert image to color space hsv
        hsv_img = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        
        # threshold the image to remove all colors that are not yellpw
        frame_threshed = cv2.inRange(hsv_img, self.COLOR[0], self.COLOR[1])
        ret,thresh = cv2.threshold(frame_threshed, self.COLOR[0][0], 255, 0)
         
        # create the list of filtered contours to return
        filtered_contours = []

        # find all contours in the thresholded image
        img, contours, hierarchy = cv2.findContours(\
                thresh,cv2.RETR_CCOMP,cv2.CHAIN_APPROX_SIMPLE)
        
        # sort all contours by area, largest to smallest
        contour_area = [ (cv2.contourArea(c), (c) ) for c in contours]
        contour_area = sorted(contour_area,reverse=True, key=lambda x: x[0])

        for j, (area,(cnt)) in enumerate(contour_area):
            # only report MAX_DETECTIONS number of controus
            if j >=self.MAX_DETECTIONS: break

            # create a bounding box around the contour
            x,y,w,h = cv2.boundingRect(cnt)
            box = (x,y,w,h)
            
            # add this contour to the list
            filtered_contours.append( (cnt, box) )
        return filtered_contours


    def contour_match(self, img):
        '''
        Returns 1. Image with bounding boxes added
        '''
        # get filtered contours
        contours = self.get_filtered_contours(img)
        detection = Detection()
        height,width,channel = img.shape
        mean_color = (15,253,250)
        for i, (cnt, box)  in enumerate(contours): 
            # plot box and label around contour
            x,y,w,h = box
            font = cv2.FONT_HERSHEY_SIMPLEX
            cv2.putText(img,"cone", (x,y), font, 1,mean_color,4)
            cv2.rectangle(img,(x,y),(x+w,y+h), mean_color,2)
            if i == 0:
                detection.x = x
                detection.y = y
                detection.w = w
                detection.h = h
                detection.error_center = 0.5 - (x/float(width))
                detection.error_size = (self.DESIRED_AREA-w*h)/float(width*height)
            cv2.putText(img,"center:%.2f, distance: %.2f" % (detection.error_center, detection.error_size), (x-w,y-h/2), font, 1,mean_color,4)
        # return the image with boxes around detected contours
        return img, detection

"""
This class is built off of Echo to call the contour_match method in 
the Detector class.  It handles the interface between ros messages
and opencv.
"""
class StaticObjectDetectorNode:
    def __init__(self):
        
        self.detector = Detector()
        self.sub_image = rospy.Subscriber("/zed/rgb/image_rect_color", Image, self.processImage, queue_size=1)
        self.pub_image = rospy.Publisher("detection_image", Image, queue_size=1)
        self.bridge = CvBridge()

        rospy.loginfo("Object Detector Initialized.")

        self.drive_cmd = {'steer': 0, 'speed': 0}

        self.pub_detection = rospy.Publisher("object_detection",\
                Detection, queue_size=1)

        #self.pub = rospy.Publisher("/vesc/ackermann_cmd_mux/input/navigation",\
        #        AckermannDriveStamped, queue_size =1 )
        #self.thread = Thread(target=self.drive)
        #self.thread.start()
        rospy.loginfo("Detector initialized")
    """
    def drive(self):
        while not rospy.is_shutdown():

            if True: #
                # to be overly explicit
                drive_msg_stamped = AckermannDriveStamped()
                drive_msg = AckermannDrive()
                drive_msg.speed = self.drive_cmd['speed']
                drive_msg.steering_angle = self.drive_cmd['steer']
                drive_msg.acceleration = 0
                drive_msg.jerk = 0
                drive_msg.steering_angle_velocity = 0
                drive_msg_stamped.drive = drive_msg
                self.pub.publish(drive_msg_stamped)
            
            # don't spin too fast
            rospy.sleep(.1)
    """

    def processImage(self, image_msg):
        # convert rosmsg to cv2 type
        image_cv = self.bridge.imgmsg_to_cv2(image_msg)            
        
        # call blob detector
        image_detections, detections  = self.detector.contour_match(image_cv)

        # convert cv2 image to rosmsg
        image_ros_msg = self.bridge.cv2_to_imgmsg(image_detections, "bgr8")

        # print len(detections)

        # publish rosmsg 
        if DEBUG: self.pub_image.publish(image_ros_msg) 
        self.pub_detection.publish(detections)
if __name__=="__main__":
    # initalize the ros node
    rospy.init_node('object_detector_node')
    
    # create the object detector
    node = StaticObjectDetectorNode()
    
    # continue running node until it is killed from outside process
    rospy.spin()
