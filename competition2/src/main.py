#! /usr/bin/env python3

import rospy
import cv2
import time
import yaml
from set_location import SetLocationClient
from bandit_solver import solve_bandit
from shapes_answer_client import ShapesAnswerClient
from movement import Movement
from path_planning_client import PathPlanningClient


# def localize():
    


# def main():
#     pass



rospy.init_node('set_location_client')
rospy.wait_for_service('/gazebo/set_model_state')
location_client = SetLocationClient()
yaml_path = '/home/user/catkin_ws/src/competition2/params/locations.yaml'
start_time = time.time()


# result = client.send_request(-9.619613649943641, 15.248309023346721, 0)

# solve kidnapped robot problem
task_time = time.time()
time_elapsed = (task_time - start_time)
print(time_elapsed)
start_time = task_time

# solve shape problem
print("skipping shape")
shape_ans_client = ShapesAnswerClient(0)
results = shape_ans_client.send_request()
next_room = results.room
how = "unknown"

task_time = time.time()
time_elapsed = (task_time - start_time)
print(time_elapsed)
start_time = task_time
# solve bandit problem
bandit_pose = yaml.safe_load(open(yaml_path))["C_centre"]
location_client.send_request(bandit_pose["position"]["x"], bandit_pose["position"]["y"], 180)
where = solve_bandit()

task_time = time.time()
time_elapsed = (task_time - start_time)
print(time_elapsed)
start_time = task_time

# solve maze problem
who = "unkown"

task_time = time.time()
time_elapsed = (task_time - start_time)
print(time_elapsed)
start_time = task_time
# accuse
print("how: " + how)
print("where: "+ where)
print("who: " + who)