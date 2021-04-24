#! /usr/bin/env python3
import rospy
from competition2.srv import MazeAnswer, MazeAnswerRequest
import sys

class MazeAnswerClient():

    def __init__(self, passcode):
        self.service = rospy.ServiceProxy('/maze_answer', MazeAnswer)
        self.passcode = passcode

    def send_request(self):
        requestObject = MazeAnswerRequest(self.passcode)
        result = self.service(requestObject)
        return result

if __name__ == "__main__":
    rospy.init_node('maze_anser_client')
    rospy.wait_for_service('/maze_answer')
    client = MazeAnswerClient(37)
    result = client.send_request()

    print(result)