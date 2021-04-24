#! /usr/bin/env python3
import rospy
from competition2.srv import ShapesAnswer, ShapesAnswerRequest
import sys

class ShapesAnswerClient():

    def __init__(self, count):
        self.service = rospy.ServiceProxy('/shapes_answer', ShapesAnswer)
        self.count = count

    def send_request(self):
        requestObject = ShapesAnswerRequest(self.count)
        result = self.service(requestObject)
        return result

if __name__ == "__main__":
    rospy.init_node('shapes_answer_client')
    rospy.wait_for_service('/shapes_answer')
    client = ShapesAnswerClient(0)
    result = client.send_request()

    print(result)