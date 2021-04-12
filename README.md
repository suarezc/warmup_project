# warmup_project

## Drive in a Square

* To drive in a square we must have our robot drive straight for a predetermined amount of time, turn 90 degrees and repeat. To accomplish this we use the /cmd_vel topic and published a twist that goes straight and then another that turns. The timing is found by experimentation.

* We have the init func that creates our node and our publisher for the /cmd_vel topic. We have our run func that creates the twist messages we need and also publishes our messages in a while loop. The main function leverages the other functions to create a node and run it and make the robot do neat things.

* ![Square Drive](./square_demonstration.gif)


## Person Follower


## Wall Follower

* To follow the wall we find the angle that is closest to the wall from the robot and turn such that our closest point is 90 degrees to the robot. Unless we are closer than our distance calls for according to a reading from the 0 and 45 degree, in which case we turn  away to get some space between the robot and the wall. When we approach a cornerthe closest angle changes and our robot turns accordingly. Then our too close code kicks in such that the robot is consistent with its distance from the wall.

* Our init function creates the publisher for the /cmd_vel topic and the subscriber for the /scan topic as well as initialize our node and a twist object as a class member. Our subscriber triggers process_scan and sends along the scan data. Our process scan will calculate the ideal angle tothe wall, typically 90 so we follow parallel but sometimes it calculates to face slightly away from the wall when we want to get some more distance between the robot and the wall. Our twist message is published here, we have a default linear speed of 0.25, but this is reduced to 0.1 when we want to move away from the way so we don't overshoot to aggressively. Our angular velocity is calculated using proportional control. The angle we want to have to our 90 degree is simply the angle that is closest to the wall, our code will correct if we get too close. After all the angular and linear velocities are assigne we publish the message to /cmd_vel. Our run method calls rospy.spin() to keep the subscribers and publishers up to date. Our main method instantiates the node and then runs it.

* ![Wall Follwer](./wall_follower_demo.gif)




