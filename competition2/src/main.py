#! /usr/bin/env python3

import rospy
import cv2

from set_location import SetLocationClient
from bandit_solver import solve_bandit
from shapes_answer_client import ShapesAnswerClient
from movement import Movement
from path_planning_client import PathPlanningClient


def localize():
    


def main():
    pass



rospy.init_node('set_location_client')
rospy.wait_for_service('/gazebo/set_model_state')
location_client = SetLocationClient()
# result = client.send_request(-9.619613649943641, 15.248309023346721, 0)

# solve kidnapped robot problem

# solve shape problem
print("skipping shape")
shape_ans_client = ShapesAnswerClient(0)
results = shape_ans_client.send_request()
next_room = results.room
how = "unknown"
# solve bandit problem
location_client.send_request(3.278423115757939, -14.265481863697897, 180)
where = solve_bandit()

# solve maze problem
who = "unkown"
# accuse
print("how: " + how)
print("where: "+ where)
print("who: " + who)