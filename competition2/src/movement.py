#! /usr/bin/env python
import rospy, math, numpy as np
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion

class VelocityController():
    def __init__(self, topic):
        self.cmd_vel = rospy.Publisher(topic, Twist, queue_size=10)
        self.max_vel = 0.65 # default: 0.65
        rospy.sleep(0.1)

    def set_max_velocity(self, new_vel):
        self.max_vel = new_vel

    def move(self, linear_velocity=0.0, angular_velocity=0.0):
        abs_v = abs(linear_velocity)
        if abs_v <= self.max_vel:
            vx = linear_velocity
            wz = angular_velocity
        else:
            vx = linear_velocity / abs_v * self.max_vel
            wz = angular_velocity / abs_v * self.max_vel
        msg = Twist()
        msg.linear.x = vx
        msg.angular.z = wz
        self.cmd_vel.publish(msg)

class OdometryReader():
    def __init__(self, topic):
        self.pose = {}
        self.trajectory = []
        self.topic = topic
        self.subscribe()

    def callback(self, msg):
        self.pose['x'] = msg.pose.pose.position.x
        self.pose['y'] = msg.pose.pose.position.y
        self.trajectory.append((self.pose['x'], self.pose['y']))
        (_, _, self.pose['theta']) = euler_from_quaternion([msg.pose.pose.orientation.x, 
                                                            msg.pose.pose.orientation.y, 
                                                            msg.pose.pose.orientation.z, 
                                                            msg.pose.pose.orientation.w])
    def subscribe(self):
        self.subscriber = rospy.Subscriber(self.topic, Odometry, self.callback)
        rospy.sleep(0.1)

    def unregister(self):
        np.save('trajectory',self.trajectory)
        self.subscriber.unregister()

class Movement():
    def __init__(self):
        self.k_rho = 0.3
        self.k_alpha = 0.8
        self.k_beta = -0.15
        self.v = 2
        self.velocity = VelocityController('/cmd_vel')
        self.odometry = OdometryReader('/odom')
    
    # close loop control
    def normalize(self, angle):
        normalized = angle%(2*math.pi)
        if normalized > math.pi:
            return  normalized - 2 * math.pi
        return normalized
    
    def go_to(self, xg, yg, thetag_degrees, rho_thresh, vel=0.65):
        self.velocity.set_max_velocity(vel)
        rho = float("inf")
        thetag = math.radians(thetag_degrees)
        while rho>rho_thresh:
            dx = xg-self.odometry.pose['x']
            dy = yg-self.odometry.pose['y']
            rho = np.sqrt(dx**2 + dy**2)
            theta = self.odometry.pose['theta']
            # theta = normalize(odometry.pose['theta'] - thetag)
            alpha = self.normalize(-theta+np.arctan2(dy, dx))
            beta = self.normalize(thetag-np.arctan2(dy, dx))
            v = self.k_rho*rho
            w = self.k_alpha*alpha + self.k_beta*beta
            self.velocity.move(v, w)
            rospy.sleep(0.01)
    
    # open loop controls
    def straight(self, s=1):
        return (float('inf'), s)

    def left(self, angle=90, radius=1):
        s = abs(radius)*abs(math.radians(angle))
        return (abs(radius), s)

    def right(self, ngle=90, radius=1):
        s = abs(radius)*abs(math.radians(angle))
        return (-abs(radius), s)

    def walkPath(self, path):
        for (R, s) in path:
                w = self.v / R
                self.velocity.move(self.v,w)
                t = s / self.v
                rospy.sleep(t) 
                # waypoints.append((odometry.pose['x'], odometry.pose['y']))


