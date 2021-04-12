#!/usr/bin/env python3

import rospy

from sensor_msgs.msg import LaserScan

from geometry_msgs.msg import Twist
from geometry_msgs.msg import Vector3

#Like to keep a meter from the wall
distance = 0.8

class WallFollower(object):

    def __init__(self):
        rospy.init_node("wall_follower")

        rospy.Subscriber("/scan", LaserScan, self.process_scan)

        self.twist_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)

        lin = Vector3()
        ang = Vector3()
        self.twist = Twist(linear=lin,angular=ang)

    def process_scan(self,data):
        #Typically, we're fine simply being 90 degrees to the wall
        ideal_angle_to_wall = 90

        #calculates the angle that is closest to the wall for positioning purposes
        angle_to_wall = data.ranges.index(min(data.ranges))

        #default speed is 0.25
        self.twist.linear.x = 0.25


        #Move away from the wall if we are too close, do so at a lower velocity to prevent overcorrection.
        if data.ranges[0] < distance + 0.1 or data.ranges[45] < distance + 0.1:
            ideal_angle_to_wall = (angle_to_wall + 90)%360
            self.twist.linear.x = 0.1


        #proportional control
        kp = 0.5
        diffang = ((ideal_angle_to_wall - angle_to_wall)/180) * 3.14
        angular = diffang * kp
        self.twist.angular.z = -angular

        

        #if robot is too far away to see wall simply move straight
        if min(data.ranges) == float("inf"):
            self.twist.angular.z = 0

        # Publish msg to cmd_vel.
        self.twist_pub.publish(self.twist)

    def run(self):
        # Keep the program alive.
        rospy.spin()

if __name__ == '__main__':
    # Declare a node and run it.
    node = WallFollower()
    node.run()
