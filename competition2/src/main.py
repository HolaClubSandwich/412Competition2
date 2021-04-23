#! /usr/bin/env python3

import rospy
import cv2

from set_location import SetLocationClient

rospy.init_node('set_location_client')
rospy.wait_for_service('/gazebo/set_model_state')
client = SetLocationClient()

result = client.send_request(-9.619613649943641, 15.248309023346721, 0)