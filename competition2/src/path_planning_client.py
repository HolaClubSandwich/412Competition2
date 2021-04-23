#! /usr/bin/env python3

import rospy
from competition2.srv import PathPlanning, PathPlanningRequest
import sys

class PathPlanningClient():

    def __init__(self, location):
        self.hitService = rospy.ServiceProxy('/path_palnning_server', PathPlanning)
        self.location = location

    def send_request(self):
        requestObject = PathPlanningRequest(self.location)
        result = self.hitService(requestObject)
        return result

if __name__ == "__main__":
    rospy.init_node('path_palnning_client')
    rospy.wait_for_service('/path_palnning_server')
    client = PathPlanningClient('lobby')
    result = client.send_request()

    print(result)