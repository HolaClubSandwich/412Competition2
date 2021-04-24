#! /usr/bin/env python3

import rospy
import time
from geometry_msgs.msg import Twist
from turtlebot_laser_class import Laser
from turtlebot_odom_client import OdomPositionClient
from maze_answer_client import MazeAnswerClient
from ocr_server import OCRDetect

class MazeSlover():

    def __init__(self):
        self.turtlebot_vel_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self.cmd = Twist()
        self.ctrl_c = False
        self.wall = False
        self.rate = rospy.Rate(10)
        self.laser = Laser()
        self.front = None
        self.left = None
        self.right = None
        self.passcode = None
        # self.odom_position_client = OdomPositionClient()
        self.ocr = OCRDetect()
        rospy.on_shutdown(self.shutdownhook)


    def publish_once_in_cmd_vel(self):
        """
        This is because publishing in topics sometimes fails the first time you publish.
        In continuous publishing systems, this is no big deal, but in systems that publish only
        once, it IS very important.
        """
        while not self.ctrl_c:
            connections = self.turtlebot_vel_publisher.get_num_connections()
            if connections > 0:
                self.turtlebot_vel_publisher.publish(self.cmd)
                # rospy.loginfo("cmd published")
                break
            else:
                self.rate.sleep()

    def shutdownhook(self):
        # self.stop_turtlebot()
        self.ctrl_c = True

    def get_regions(self):
        regions = self.laser.get_laser_for_maze()
        self.front = regions['front']
        self.left = regions['left']
        self.right = regions['right']

    # def check_position(self):
    #     result = self.odom_position_client.send_request()
    #     if (result.position > self.end_x):
    #         return True

    def stop_bot(self):
        self.cmd.linear.x = 0.0
        self.cmd.angular.z = 0.0
        self.publish_once_in_cmd_vel()

    def move(self):
        print("Going into maze")
        self.cmd.linear.x = 0.5
        self.turtlebot_vel_publisher.publish(self.cmd)
        time.sleep(2)
        # self.stop_bot()
        i = 0

        while not self.ctrl_c:
            self.get_regions()

            # if self.check_position():
                # self.stop_bot()
                # self.ctrl_c = True

            while (not self.wall and not self.ctrl_c):
                self.get_regions()
                print("moving towards a wall")
                print(self.front)
                if (self.front > 0.35 and self.right > 0.35 and self.left > 0.35):
                    self.cmd.angular.z = 0.3
                    self.cmd.linear.x = 0.4

                elif (self.right < 0.35):
                    self.wall = True
                else:
                    self.cmd.angular.z = 0.3
                    self.cmd.linear.x = 0.0
                self.publish_once_in_cmd_vel()
            
            else:
                print("right wall detected")
                print(self.front)
                if (self.front > 0.38):
                    if (self.right < 0.25):
                        print("Distance: {:.2f}m - Too close. Backing up".format(self.right))
                        self.cmd.angular.z = 0.8
                        self.cmd.linear.x = -0.1
                    elif (self.right > 0.35):
                        print("Distance: {:.2f}m - Following wall, turn right".format(self.right))
                        self.cmd.angular.z = -1.0
                        self.cmd.linear.z = 0.5
                    else:
                        print("Distance: {:.2f}m - Following wall, turn left".format(self.right))
                        self.cmd.angular.z = 1.0
                        self.cmd.linear.x = 0.5
                else:
                    print("Front wall detected, turning around.")
                    print(self.front)
                    # self.stop_bot()
                    # self.ctrl_c = True
                    self.cmd.angular.z = 0.8
                    self.cmd.linear.x = 0.0
                    self.publish_once_in_cmd_vel()
                    while(self.front < 0.4 and not self.ctrl_c):
                        self.get_regions()
                self.publish_once_in_cmd_vel()
                # self.stop_bot()
                # self.ctrl_c = True
            i += 1
            print(i)
            self.rate.sleep()

        who, next_room = self.read_sign()
        return who, next_room

    def read_sign(self):
        detect_text = self.ocr.read_sign('/home/user/catkin_ws/src/competition2/models/end/one.png')
        print(detect_text)
        lines = detect_text.split("\n")
        for line in lines:
            if "Passcode" in line:
                words = line.split(":")
                self.passcode = int(words[1].strip())
        print("passcode: " + str(self.passcode))

        maze_client = MazeAnswerClient(self.passcode)
        result = maze_client.send_request()
        next_room = result.room
        who = result.who
        # print(result)
        # print(who)
        return who, next_room

    

    
if __name__ == "__main__":
    rospy.init_node('maze_test')
    # rospy.wait_for_service('/odom_position')
    test = MazeSlover()
    test.move()
    # test.read_sign()