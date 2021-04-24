#! /usr/bin/env python

import rospy
import cv2
from pytesseract import image_to_string

class OCRDetect():

    def __init__(self):
        pass

    def read_sign(self, path):
        img = cv2.imread(path, 1)
        detect_text = image_to_string(img)
        # print(detect_text)

        return detect_text

    def read_map(self, path):
        img = cv2.imread(path, cv2.IMREAD_GRAYSCALE)

        img[img != 255] = 0

        thresh = cv2.threshold(img, 100, 255, cv2.THRESH_BINARY_INV + cv2.THRESH_OTSU)[1]
        thresh = cv2.resize(thresh, (0, 0), fx=2, fy=2)

        detect_text = image_to_string(thresh, config='--psm 6')

        return detect_text
    
