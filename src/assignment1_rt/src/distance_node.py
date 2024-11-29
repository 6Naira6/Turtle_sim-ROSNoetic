#!/usr/bin/env python3

import rospy
from turtlesim.msg import Pose
from std_msgs.msg import Float32
from geometry_msgs.msg import Twist
import math

class DistanceChecker:
    def __init__(self):
        rospy.init_node('node2_distance', anonymous=True)

        self.pose_turtle1 = None
        self.pose_turtle2 = None

        rospy.Subscriber('/turtle1/pose', Pose, self.update_pose_turtle1)
        rospy.Subscriber('/turtle2/pose', Pose, self.update_pose_turtle2)

        self.pub_distance = rospy.Publisher('/turtles_distance', Float32, queue_size=10)
        self.pub_stop_turtle2 = rospy.Publisher('/turtle2/cmd_vel', Twist, queue_size=10)

        self.threshold_distance = 1.0
        self.boundary_limit = (1.0, 10.0)

    def update_pose_turtle1(self, data):
        self.pose_turtle1 = data

    def update_pose_turtle2(self, data):
        self.pose_turtle2 = data

    def compute_distance(self):
        if self.pose_turtle1 and self.pose_turtle2:
            dx = self.pose_turtle1.x - self.pose_turtle2.x
            dy = self.pose_turtle1.y - self.pose_turtle2.y
            return math.sqrt(dx**2 + dy**2)
        return None

    def monitor(self):
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            distance = self.compute_distance()
            if distance is not None:
                self.pub_distance.publish(distance)
                if distance < self.threshold_distance or self.is_near_boundary(self.pose_turtle2):
                    rospy.logwarn("Stopping turtle2 due to proximity or boundary!")
                    self.pub_stop_turtle2.publish(Twist())

            rate.sleep()

    def is_near_boundary(self, pose):
        return pose.x < self.boundary_limit[0] or pose.x > self.boundary_limit[1] or \
               pose.y < self.boundary_limit[0] or pose.y > self.boundary_limit[1]

if __name__ == '__main__':
    try:
        checker = DistanceChecker()
        checker.monitor()
    except rospy.ROSInterruptException:
        pass

