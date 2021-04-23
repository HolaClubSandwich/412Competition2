#! /usr/bin/env python3

import rospy
import time
from geometry_msgs.msg import Twist
from turtlebot_laser_class import Laser

class MazeSlover():

    def __init__(self):
        self.turtlebot_vel_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self.cmd = Twist()
        self.ctrl_c = False
        self.wall = False
        self.rate = rospy.Rate(1)
        self.laser = Laser()
        self.front = None
        self.left = None
        self.right = None
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

        while not self.ctrl_c:
            self.get_regions()

            while (not self.wall and not self.ctrl_c):
                self.get_regions()
                print("moving towards a wall")
                print(self.front)
                if (self.front > 0.35 and self.right > 0.3 and self.left > 0.3):
                    self.cmd.angular.z = 0.3
                    self.cmd.linear.x = 0.4
                    # print('x')
                elif (self.right < 0.35):
                    self.wall = True
                else:
                    self.cmd.angular.z = 0.3
                    self.cmd.linear.x = 0.0
                self.publish_once_in_cmd_vel()
            
            else:
                print("right wall detected")
                print(self.front)
                if (self.front > 0.35):
                    if (self.right < 0.2):
                        print("Range: {:.2f}m - Too close. Backing up".format(self.right))
                        self.cmd.angular.z = 1.2
                        self.cmd.linear.x = -0.6
                    elif (self.right > 0.2):
                        print("Range: {:.2f}m - Following wall, turn right".format(self.right))
                        self.cmd.angular.z = -1.2
                        self.cmd.linear.z = 0.6
                    else:
                        print("Range: {:.2f}m - Following wall, turn left".format(self.right))
                        self.cmd.angular.z = 1.2
                        self.cmd.linear.x = 0.6
                else:
                    print("Front wall detected, turning around.")
                    print(self.front)
                    # self.stop_bot()
                    # self.ctrl_c = True
                    self.cmd.angular.z = 1.0
                    self.cmd.linear.x = 0.0
                    self.publish_once_in_cmd_vel()
                    while(self.front < 0.45 and not self.ctrl_c):
                        self.get_regions()
                self.publish_once_in_cmd_vel()
                # self.stop_bot()
                # self.ctrl_c = True
    

    
if __name__ == "__main__":
    rospy.init_node('maze_test')
    test = MazeSlover()
    test.move()