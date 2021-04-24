#! /usr/bin/env python
import rospy
from gazebo_msgs.srv import SetModelState, SetModelStateRequest
from movement import Movement
import sys

class SetLocationClient():
    def __init__(self):
        self.setLocationService = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
    
    def send_request(self, x, y, d):
        requestObject = SetModelStateRequest()
        requestObject.model_state.model_name = "mobile_base"
        requestObject.model_state.pose.position.x = x
        requestObject.model_state.pose.position.y = y
        requestObject.model_state.pose.orientation.z = d
        result = self.setLocationService(requestObject)
        return result

if __name__ == "__main__":
    
    rospy.init_node('set_location_client')
    rospy.wait_for_service('/gazebo/set_model_state')
    client = SetLocationClient()
    # result = client.send_request(-0.6, -8.1, -180)
    # for location3
#     C:
#   position:
#     x: -1.492526754245629
#     y: -15.458635248810356
#     z: 0.0
#   orientation:
#     x: 0.0
#     y: 0.0
#     z: -0.06459642077176161
#     w: 0.9979114702334458
    result = client.send_request(-1.492526754245629, -15.458635248810356, -0.06459642077176161)