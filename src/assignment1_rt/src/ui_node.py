#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from turtlesim.srv import Spawn

def spawn_turtle():
    rospy.wait_for_service('/spawn')
    try:
        spawn = rospy.ServiceProxy('/spawn', Spawn)
        spawn(5.0, 5.0, 0.0, 'turtle2')
    except rospy.ServiceException as e:
        rospy.logerr(f"Service call failed: {e}")

def main():
    rospy.init_node('node1_ui', anonymous=True)
    pub_turtle1 = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)
    pub_turtle2 = rospy.Publisher('/turtle2/cmd_vel', Twist, queue_size=10)

    spawn_turtle()

    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        print("\nControl Menu:")
        print("1. Turtle1")
        print("2. Turtle2")
        choice = input("Select turtle (1 or 2): ")
        try:
            choice = int(choice)
            if choice not in [1, 2]:
                print("Invalid choice.")
                continue
        except ValueError:
            print("Please enter a valid number.")
            continue

        vel_msg = Twist()
        vel_msg.linear.x = float(input("Linear velocity: "))
        vel_msg.angular.z = float(input("Angular velocity: "))

        publisher = pub_turtle1 if choice == 1 else pub_turtle2
        for _ in range(10):
            publisher.publish(vel_msg)
            rate.sleep()

        publisher.publish(Twist())

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass

