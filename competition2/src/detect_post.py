#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np
import matplotlib.pyplot as plt


class DetectPPL(object):

    def __init__(self):
    
        self.image_sub = rospy.Subscriber("/camera/rgb/image_raw",Image,self.camera_callback)
        self.bridge_object = CvBridge()



    def camera_callback(self,data):
        try:
            # We select bgr8 because its the OpenCV encoding by default
            cv_image = self.bridge_object.imgmsg_to_cv2(data, desired_encoding="bgr8")
        except CvBridgeError as e:
            print(e)
        
        photo = cv_image

        #Reduce photo size to reduce sensitivity
        photo = cv2.resize(photo,(600, 400))
        gray_photo = cv2.cvtColor(photo, cv2.COLOR_RGB2GRAY)
        # ret,thresh = cv2.adaptiveThreshold(gray_photo,255,cv2.ADAPTIVE_THRESH_MEAN_C, cv2.THRESH_BINARY,11,2)

        blur = cv2.GaussianBlur(gray_photo,(5,5),0)
        ret,thresh = cv2.threshold(blur,0,255,cv2.THRESH_BINARY+cv2.THRESH_OTSU)
        cv2.imshow("Thresh", thresh)
        cv2.imshow('Detection',photo)       

        cv2.waitKey(1)   


def main():
    detect_wanted_object = DetectPPL()
    rospy.init_node('detect_wanted_node', anonymous=True)
    
    try:
        rospy.spin()
        
    except KeyboardInterrupt:
        print("Shutting down")
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()