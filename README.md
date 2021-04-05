# warmup_project

## Drive in a Square

* To drive in a square we must have our robot drive straight for a predetermined amount of time, turn 90 degrees and repeat. To accomplish this we use the /cmd_vel topic and published a twist that goes straight and then another that turns. The timing is found by experimentation.

* We have the init func that creates our node and our publisher for the /cmd_vel topic. We have our run func that creates the twist messages we need and also publishes our messages in a while loop. The main function leverages the other functions to create a node and run it and make the robot do neat things.

* ![Square Drive](/square_demonstration.gif)
