#! /usr/bin/env python3

import rospy
from competition2.srv import OdomPosition, OdomPositionRequest

class OdomPositionClient():

    def __init__(self):
        self.hitService = rospy.ServiceProxy('/odom_position', OdomPosition)

    def send_request(self):
        requestObject = OdomPositionRequest()
        result = self.hitService(requestObject)
        return result


if __name__ == "__main__":
    rospy.init_node('odom_position_test')
    rospy.wait_for_service('/odom_position')
    client = OdomPositionClient()
    result = client.send_request()
    print(type(result.position))