#!/usr/bin/env python3

import rospy
# msgs needed for /cmd_vel
from geometry_msgs.msg import Twist, Vector3

class DriveSquare(object):
    """ This node makes the cute robot go vroom in a square """

    def __init__(self):
        # initialize the ROS node
        rospy.init_node('drive_square')
        # setup publisher to the cmd_vel ROS topic
        self.robot_movement_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

    def run(self):
        # setup the Twist message we want to send for going straight
        straight_twist = Twist(
            linear=Vector3(0.2, 0, 0),
            angular=Vector3(0, 0, 0.0)
        )
        #set up our twist for the turn
        turn_twist = Twist(
            linear=Vector3(0.0, 0, 0),
            angular=Vector3(0, 0, 0.4)
        )

        # allow the publisher enough time to set up before publishing the first msg
        rospy.sleep(1) 

        # drive in a square forever
        while True:
            self.robot_movement_pub.publish(straight_twist)
            #allow time for forward
            rospy.sleep(4)
            self.robot_movement_pub.publish(turn_twist)
            #allow time for turn
            rospy.sleep(4)


if __name__ == '__main__':
    # instantiate the ROS node and run it
    node = DriveSquare()
    node.run()