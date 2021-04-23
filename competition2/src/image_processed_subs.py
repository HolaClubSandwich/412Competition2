#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
import matplotlib.pyplot as plt
import math
import cv2
from pytesseract import image_to_string



class IMGProcessedSubscriber(object):

    def __init__(self):
    
        self.image_sub = rospy.Subscriber("/camera/rgb/image_raw",Image,self.camera_callback)
        self.bridge_object = CvBridge()
        self.image = None
        self.gray_image = None


    def camera_callback(self,data):
        try:
            # We select bgr8 because its the OpenCV encoding by default
            cv_image = self.bridge_object.imgmsg_to_cv2(data, desired_encoding="bgr8")
        except CvBridgeError as e:
            print(e)
        # cv2.imshow("test_image", cv_image)


        photo = cv_image

        #Reduce photo size to reduce sensitivity
        # photo = cv2.resize(photo,(600, 400))
        gray_photo = cv2.cvtColor(photo, cv2.COLOR_RGB2GRAY)
        thresh = cv2.threshold(gray_photo, 100, 255, cv2.THRESH_BINARY_INV + cv2.THRESH_OTSU)[1]
        thresh = cv2.resize(thresh, (0, 0), fx=2, fy=2)
        self.image = photo
        self.gray_image = gray_photo
        


def main():
    img_process_object = IMGProcessedSubscriber()
    rospy.init_node('img_process_node', anonymous=True)
    
    try:
        rospy.spin()
        
    except KeyboardInterrupt:
        print("Shutting down")
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()