#!/usr/bin/env python3

import rospy

from sensor_msgs.msg import LaserScan

from geometry_msgs.msg import Twist
from geometry_msgs.msg import Vector3

distance = 0.75

class PersonFollower(object):

    def __init__(self):
        rospy.init_node("person_follower")

        rospy.Subscriber("/scan", LaserScan, self.process_scan)

        self.twist_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)

        lin = Vector3()
        ang = Vector3()
        self.twist = Twist(linear=lin,angular=ang)

    def process_scan(self,data):

        #calculate the angle we need to face in order to face the closest object to the robot
        angle_to_face = data.ranges.index(min(data.ranges))

        #turn left if it will be closer
        if angle_to_face > 180:
            angle_to_face = angle_to_face - 360
        #proportional control
        kp = 0.5
        diff = ((0 - angle_to_face)/180) * 3.14
        angular = diff * kp

    
        self.twist.angular.z = -angular

        #only move if the robot detects something, is far away enough and if it is facing the object within reason
        if (min(data.ranges) >= distance and min(data.ranges) != float("inf")) and (angle_to_face > -30 and angle_to_face < 30):
            #move if not close
            self.twist.linear.x = 0.3
        else:
            # Close enough, stop.
            self.twist.linear.x = 0
        # Publish msg to cmd_vel.
        self.twist_pub.publish(self.twist)

    def run(self):
        # Keep the program alive.
        rospy.spin()

if __name__ == '__main__':
    # Declare a node and run it.
    node = PersonFollower()
    node.run()
