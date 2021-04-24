#! /usr/bin/env python3
import rospy
from competition2.srv import BanditAnswer, BanditAnswerRequest
import sys

class BanditAnswerClient():

    def __init__(self, arm):
        self.service = rospy.ServiceProxy('/bandit_answer', BanditAnswer)
        self.arm = arm

    def send_request(self):
        requestObject = BanditAnswerRequest(self.arm)
        result = self.service(requestObject)
        return result

if __name__ == "__main__":
    rospy.init_node('bandit_anser_client')
    rospy.wait_for_service('/bandit_answer')
    client = BanditAnswerClient(1)
    result = client.send_request()

    print(result)