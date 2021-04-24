#! /usr/bin/env python3
import rospy
from competition2.srv import BanditStep, BanditStepRequest
import sys

class BanditStepClient():

    def __init__(self, passcode):
        self.service = rospy.ServiceProxy('/bandit_step', BanditStep)
        self.passcode = passcode

    def send_request(self, action):
        requestObject = BanditStepRequest()
        requestObject.passcode = self.passcode
        requestObject.action = action
        result = self.service(requestObject)
        return result

if __name__ == "__main__":
    rospy.init_node('bandit_step_client')
    rospy.wait_for_service('/bandit_step')
    client = BanditStepClient(42)
    result = client.send_request(1)

    print(result)