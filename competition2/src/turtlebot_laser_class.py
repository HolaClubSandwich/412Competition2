#!/usr/bin/env python3

import rospy
import time
from sensor_msgs.msg import LaserScan

class Laser():

    def __init__(self):
        self.laser_subcriber = rospy.Subscriber('/kobuki/laser/scan', LaserScan, self.laser_callback)
        self.laser_msg = LaserScan()

    def laser_callback(self, msg):
        self.laser_msg = msg

    def get_laser(self, position):
        time.sleep(1)
        return self.laser_msg.ranges[position]

    def get_laser_front(self):
        time.sleep(1)
        return self.laser_msg.ranges[360]

    def get_laser_all(self):
        time.sleep(1)
        return self.laser_msg.ranges

    def get_laser_for_maze(self):
        time.sleep(1)
        regions = {'right': min(min(self.laser_msg.ranges[0:143]), 10), 'fright': min(min(self.laser_msg.ranges[144:287]), 10), 'front': min(min(self.laser_msg.ranges[288:431]), 10), 'fleft': min(min(self.laser_msg.ranges[432:575]), 10), 'left': min(min(self.laser_msg.ranges[576:713]), 10)}
        return regions

if __name__ == "__main__":
    rospy.init_node('laser_test', anonymous=True)
    test_laser = Laser()
    try:
        laser1 = test_laser.get_laser_for_maze()
        print("test1: ", laser1)

    except rospy.ROSInterruptException:
        pass