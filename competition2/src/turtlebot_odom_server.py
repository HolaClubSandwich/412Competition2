#! /usr/bin/env python3

import rospy
from nav_msgs.msg import Odometry
from competition2.srv import OdomPosition, OdomPositionResponse
import time

class OdomServer():

    _response = OdomPositionResponse()

    def __init__(self):
        self.odometry_subcriber = rospy.Subscriber('/odom', Odometry, self.odometry_callback)
        self.server = rospy.Service('/odom_position', OdomPosition, self.my_callback)
        self.odometry_msg = Odometry()

    def odometry_callback(self, msg):
        self.odometry_msg = msg

    def my_callback(self, request):
        time.sleep(1)
        position = self.odometry_msg.pose.pose.position.x
        self._response.position = position
        return self._response


if __name__ == "__main__":
    rospy.init_node('odom_position_server')
    server = OdomServer()
    rospy.spin()

