# warmup_project

## Drive in a Square

* To drive in a square we must have our robot drive straight for a predetermined amount of time, turn 90 degrees and repeat. To accomplish this we use the /cmd_vel topic and published a twist that goes straight and then another that turns. The timing is found by experimentation.

* We have the init func that creates our node and our publisher for the /cmd_vel topic. We have our run func that creates the twist messages we need and also publishes our messages in a while loop. The main function leverages the other functions to create a node and run it and make the robot do neat things.

* ![Square Drive](./square_demonstration.gif)


## Person Follower

* To follow the person/cylinder, we simply take the results of the lidar scan to find which direction we must face, we turn towards that direction, and once we are facing that object we move forward torwards it. We are constantly updating the scan results so if the object moves we can turn to face it again and move forward.

* Our init functions creates the publisher for the /cmd_vel topic and the subscriber to the /scan
topic, as well as initialize our node and twist class member. The scan subscriber calls process_scan with the scan data and here is where our movement is calculated. We calculate the angle we need to face by finding the degree that corresponds to the closest object. We turn to face that object using proportional control and this becomes our angular movement. We move forward only if we are not too close based on the distance we set, but also if we are roughly facing the object, it doesn't need to be dead on but we don't want to move away from the object and potentially lose it. If we are close enough we stop moving forward. Our run method is what keeps our program updating by running rospy.spin(), and our main method creates our node and runs it.

* ![Person Follower](./wall_follower_demo.gif)


## Wall Follower

* To follow the wall we find the angle that is closest to the wall from the robot and turn such that our closest point is 90 degrees to the robot. Unless we are closer than our distance calls for according to a reading from the 0 and 45 degree, in which case we turn  away to get some space between the robot and the wall. When we approach a corner the closest angle changes and our robot turns accordingly. Then our too close code kicks in such that the robot is consistent with its distance from the wall.

* Our init function creates the publisher for the /cmd_vel topic and the subscriber for the /scan topic as well as initialize our node and a twist object as a class member. Our subscriber triggers process_scan and sends along the scan data. Our process scan will calculate the ideal angle to the wall, typically 90 so we follow parallel but sometimes it calculates to face slightly away from the wall when we want to get some more distance between the robot and the wall. Our twist message is published here, we have a default linear speed of 0.25, but this is reduced to 0.1 when we want to move away from the way so we don't overshoot to aggressively. Our angular velocity is calculated using proportional control. The angle we want to have to our 90 degree is simply the angle that is closest to the wall, our code will correct if we get too close. After all the angular and linear velocities are assigned we publish the message to /cmd_vel. Our run method calls rospy.spin() to keep the subscribers and publishers up to date. Our main method instantiates the node and then runs it.

* ![Wall Follwer](./wall_follower_demo.gif)



## Challenges

The drive in the square was easy enough, the timing was a bit trial and error. The person follower was fine except for when we wanted to move towards the object in question. At first it would only move if it was exactly facing the object, but this proved to be cumbersome and impractical, so the conditon to move if it was roughly facing the right direction was added. I had some trouble with the wall follower, trying to reason when the wall angle should change and how we implement that behavior into an angular movement was tricky. The ideal_angle used to not exist and was just the number 90, but this proved to be troblesome because the robot would always get closer to the wall and eventually would touch the way and stop responding. The dynamic ideal_angle_to_wall allows the robot to correct itself and not get too close to the wall. It used to only respond to when the 0 degree measurement was too close, but this was also causing problems with bumping into the wall and checking the 45 degree spot fixed this issue.

## Future Work

The wall follower when positioned far away from the wall or even when close to a way may do a little twirl or two as it navigates close enough to a wall to behave properly. With more time I'd like to remove the twirling, despite the stylish flair it adds to the program. The person follower moves a bit oddly, only moving forward when it is facing and stopping when necessary to face another direction. I'd like it to be more dynamic and smooth, updating direction while also retaining some movement not stopping entirely to turn. I'd like to refine the PID control scheme used in these programs, perhaps play around with the multiplier for the error to find something better. In wall follower I'd like to implement proportional control for the linear velocity as well, it wasn't necessary to make it work but it may make the robot move more assuredly.

## Takeaways

* Playing around with angles can be tricky. Calculating the exact angles you want your robot to face or face relative to can be troublesome and noise + other environmental factors can make moving your robot precisely difficult. To alleivate this, it pays to take sensor data from other nearby sources/angles and programming your robot to operate within a range of acceptable parameters not just a single point or angle.

* Turning and changing direction can be slow, so moving earlier can be better. In order to keep a certain distance from an object or some other goal, having your robot begin changing its behavior before it hits a distance can mean a smoother transition to whatever movement is required and can do a better job at maintaining a distance or position than simply moving at the last second. I suppose this is what PID control is all about.


