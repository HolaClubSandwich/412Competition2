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
        
    def draw_box(self, img, box):
        (xA, yA, xB, yB) = box
        #Center in X 
        medX = xB - xA 
        xC = int(xA+(medX/2)) 

        #Center in Y
        medY = yB - yA 
        yC = int(yA+(medY/2)) 

        #Draw a circle in the center of the box 
        cv2.circle(img,(xC,yC), 1, (0,255,255), -1)

        # display the detected boxes in the original picture
        cv2.rectangle(img, (xA, yA), (xB, yB),
                            (255, 255, 0), 2)  



    def camera_callback(self,data):
        try:
            # We select bgr8 because its the OpenCV encoding by default
            cv_image = self.bridge_object.imgmsg_to_cv2(data, desired_encoding="bgr8")
        except CvBridgeError as e:
            print(e)
        
        # cv2.imwrite("/home/user/catkin_ws/src/project/src/photo.jpg", cv_image)
        wanted = cv2.imread('/home/user/catkin_ws/src/project/src/photo.jpg',1)
        photo = cv_image

        # #Reduce photo size to reduce sensitivity
        # # wtdX, wtdY, _ = wanted.shape
        # # wanted = cv2.resize(wanted, (wtdX//10, wtdY//10))
        # # phX, phY, _ = photo.shape
        # photo = cv2.resize(photo,(600, 400))
        # # print("here")
        # #Get people detection boxes
        # boxes, weights = self.hog.detectMultiScale(photo, winStride=(8,8))
        # boxes = np.array([[x, y, x + w, y + h] for (x, y, w, h) in boxes])

        # for box in boxes:
        #     self.draw_box(photo, box)
          

        # # cv2.imshow('Points',preview_wanted)
        # cv2.imshow("matching lines", dots)
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