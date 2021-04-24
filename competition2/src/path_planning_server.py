#! /usr/bin/env python3

import rospy
import yaml
from competition2.srv import PathPlanning, PathPlanningResponse
from move_base_msgs.msg import MoveBaseActionGoal
from geometry_msgs.msg import Pose


class PathPlanningServer():

    def __init__(self):
        self.server = rospy.Service('/path_palnning_server', PathPlanning, self.my_callback)
        self.goal_pub = rospy.Publisher('/move_base/goal', MoveBaseActionGoal, queue_size=1)
        self._response = PathPlanningResponse()
        self.goal = MoveBaseActionGoal()
        self.location = None
        self.curr_goal = Pose()

    def my_callback(self, request):
        self.location = request.location
        path = '/home/user/catkin_ws/src/competition2/params/locations.yaml'
        pose = yaml.safe_load(open(path))[self.location]
        self.curr_goal.position.x = pose["position"]["x"]
        self.curr_goal.position.y = pose["position"]["y"]
        self.curr_goal.position.z = pose["position"]["z"]
        self.curr_goal.orientation.x = pose["orientation"]["x"]
        self.curr_goal.orientation.y = pose["orientation"]["y"]
        self.curr_goal.orientation.z = pose["orientation"]["z"]
        self.curr_goal.orientation.w = pose["orientation"]["w"]
        self.goal.goal.target_pose.pose = self.curr_goal
        self.goal.goal.target_pose.header.frame_id = 'map'
        self.goal_pub.publish(self.goal)
        self._response.result = True
        self._response.message = "Location sent."

        return self._response

if __name__ == "__main__":
    rospy.init_node('path_planning_server_node')
    path_planning_server = PathPlanningServer()
    rospy.spin()
