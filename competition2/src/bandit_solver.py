#!/usr/bin/env python3
import rospy
import numpy as np
import random
from ocr_server import OCRDetect
from bandit_step_client import BanditStepClient
from bandit_answer_client import BanditAnswerClient
import collections

random.seed(0)

# read the picture for passcode and info
def get_info():
    ocrDetect = OCRDetect()
    detect_text = ocrDetect.read_sign('/home/user/catkin_ws/src/competition2/models/bandit/one.png')
    lines = detect_text.split("\n")
    passcode = 0
    number_arms = 0
    for line in lines:
        if "Passcode" in line:
            words = line.split(":")
            passcode = int(words[1].strip())
        if "Number arms" in line:
            words = line.split(":")
            number_arms = int(words[1].strip())
    return passcode, number_arms


def find_opt_arm(passcode, number_arms, epoch):
    # try pulling the arms for reward
    rospy.wait_for_service('/bandit_step')
    Q_table = np.zeros([number_arms])
    N_table = np.zeros([number_arms])
    step_client = BanditStepClient(passcode)
    epsilon = 0.1
    for i in range(epoch):
        # change action selection to UCB if we have time
        rand = random.uniform(0,1)
        if (rand < epsilon):
            action = random.randint(1, number_arms)
        else:
            action = np.argmax(Q_table)
            action +=1 # index to action
        results = step_client.send_request(action)
        if not results.valid:
            print("Something went wrong with the actions")
            break
        reward = results.reward
        N_table[action-1] += 1
        Q_table[action-1] += (1/N_table[action-1])*(reward - Q_table[action-1])
        # print(action, reward, Q_table)
        
    # decide the bese arm to pull
    print(Q_table)
    best_arm = int(np.argmax(Q_table))+1
    print("best arm: "+ str(best_arm))
    dictionary = {}
    for i in range(Q_table.shape[0]):
        dictionary[i+1] = Q_table[i]
    
    ranking = {k: v for k, v in sorted(dictionary.items(), key=lambda item: item[1], reverse=True)}
    ranking_string = ""
    for r in ranking.keys():
        ranking_string += str(r)+" "
    print("arm ranking: " + ranking_string)
    return best_arm



def solve_bandit():
    passcode, number_arms = get_info()
    # ## for testing purpose
    # passcode = 23
    # number_arms=5
    print("passcode: " + str(passcode))
    print("number_arms: " + str(number_arms))
    # epoch = 10**number_arms
    epoch = number_arms * 100
    while True:
        best_arm = find_opt_arm(passcode, number_arms, epoch)
        # get the next room number
        answerClient = BanditAnswerClient(best_arm)
        result = answerClient.send_request()
        next_room = result.room
        where = result.where
        print("next room: "+ str(next_room))
        print("where: "+ where)
        if where == '':
            print("did not get clue, running again with twice the epoch")
            epoch = epoch*2
        else:
            break
    return where, next_room




if __name__ == "__main__":
    where = solve_bandit()
