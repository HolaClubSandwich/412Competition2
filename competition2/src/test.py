#! /usr/bin/env python3
import cv2
from pytesseract import image_to_string
import numpy as np
from ocr_server import OCRDetect

print("hello world")
ocrDetect = OCRDetect()


# img = cv2.imread('/home/user/catkin_ws/src/competition2/models/map/one1.png', 1) #, cv2.IMREAD_GRAYSCALE)

detect_text = ocrDetect.read_sign('/home/user/catkin_ws/src/competition2/models/lobby/one.png')

# img = cv2.resize(img, (500, 500))
# gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
# new_img = np.zeros(img.shape, img.dtype)
# for y in range(img.shape[0]):
#     for x in range(img.shape[1]):
#         for z in range(img.shape[2]):
#             new_img[y, x, z] = np.clip(3.0*img[y, x, z] + 0, 0, 255)

# gray_photo = cv2.cvtColor(new_img, cv2.COLOR_RGB2GRAY)


# for i in range(100000000):
#     cv2.imshow("test", img)
# img2 = cv2.imread()
# img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
'''
img[img != 255] = 0

thresh = cv2.threshold(img, 100, 255, cv2.THRESH_BINARY_INV + cv2.THRESH_OTSU)[1]
thresh = cv2.resize(thresh, (0, 0), fx=2, fy=2)
'''
# cv2.imwrite('/home/user/catkin_ws/src/competition2/src/test.png', thresh)

# detect_text = image_to_string(thresh, config='--psm 6')

# detect_text = image_to_string(img)

print(detect_text)

if 'highest' in detect_text:
    print('TRUE')
