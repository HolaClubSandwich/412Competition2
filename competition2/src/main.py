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
from ocr_server import OCRDetect
from movement import VelocityController, Movement
from maze_solver import MazeSlover


def localize():
    move = VelocityController('/cmd_vel')
    move.move(0.65, 0)
    time.sleep(3)
    move.move(0, 0.8)
    time.sleep(10)
    move.move(0, 0)
    





rospy.init_node('set_location_client')
# rospy.wait_for_service('/gazebo/set_model_state')
location_client = SetLocationClient()
yaml_path = '/home/user/catkin_ws/src/competition2/params/locations.yaml'

start_time = time.time()

is_localized = False
while not is_localized:
    localize()
    local = input('Is localized? Y/N: ')
    if local == 'y' or local == 'Y':
        is_localized = True


lobby_path = PathPlanningClient('lobby')
lobby_path.send_request()
input('Press enter to conitue when robot arrives at lobby')

# init OCRDetect
ocr = OCRDetect()


# read lobby sign
highest = False
lowest = False
lobby_text = ocr.read_sign('/home/user/catkin_ws/src/competition2/models/lobby/one.png')
if 'highest' in lobby_text:
    highest = True
    print("highest")
elif 'lowest' in lobby_text:
    lowest = True
    print('lowest')


# lobby read mpa
map_dict = {}
room_list = ['A', 'E', 'I', 'O', 'K', 'F', 'J', 'B', 'G', 'L', 'P', 'C', 'lobby', 'M', 'D', 'H', 'N', 'Q', 'last']
detect_map = ocr.read_map('/home/user/catkin_ws/src/competition2/models/map/one.png')
# print(detect_map)
lines = detect_map.split("\n")
i = 0
for line in lines:
    numbers = line.split(" ")
    for number in numbers:
        map_dict[room_list[i]] = number
        i += 1
del(map_dict['last'])
del(map_dict['lobby'])
print("Room Assignments")
print(map_dict)

key_list = list(map_dict.keys())
temp_list = list(map_dict.values())
val_list = []
for i in temp_list:
    val_list.append(int(i))

if highest:
    first_room_num = max(val_list)
elif lowest:
    first_room_num = min(val_list)

print(first_room_num)

first_room_position = val_list.index(first_room_num)
first_room_name = key_list[first_room_position]

print(first_room_name)

input('Press enter to continue')


first_room_path = PathPlanningClient(first_room_name + '_centre')
first_room_path.send_request()
print("Travelling to the first room...")

input('Press enter to conitue when robot arrives.')





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
print("teleporting to bandit room")
bandit_room_position = val_list.index(next_room)
bandit_room_name = key_list[bandit_room_position]

bandit_pose = yaml.safe_load(open(yaml_path))[bandit_room_name + '_centre']
location_client.send_request(bandit_pose["position"]["x"], bandit_pose["position"]["y"], 180)
where, maze_room = solve_bandit()

task_time = time.time()
time_elapsed = (task_time - start_time)
print(time_elapsed)
start_time = task_time

# solve maze problem
print("teleporting to maze room")
maze_room_position = val_list.index(maze_room)
maze_room_name = key_list[maze_room_position]
maze_pose = yaml.safe_load(open(yaml_path))[maze_room_name]
location_client.send_request(maze_pose["position"]["x"], maze_pose["position"]["y"], 0)
maze = MazeSlover()
who, last_room = maze.move()



task_time = time.time()
time_elapsed = (task_time - start_time)
print(time_elapsed)


start_time = task_time
last_room_position = val_list.index(last_room)
print("teleporting to last room")
last_room_name = key_list[last_room_position]
last_pose = yaml.safe_load(open(yaml_path))[last_room_name]
location_client.send_request(last_pose["position"]["x"], last_pose["position"]["y"], 0)
# accuse
print("how: " + how)
print("where: "+ where)
print("who: " + who)
