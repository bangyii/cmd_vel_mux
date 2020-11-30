# cmd_vel_mux

Command velocity multiplexer designed to switch velocity that is sent to the robot (from autonomous local planner to shared control planner or vice versa), depending on whether the user has an input velocity on the joystick.

## Subscribers

`/joy_vel [geometry_msgs::Twist]:` User's joystick input twist

`/shared_dwa/cmd_vel [geometry_msgs::Twist]:` Shared control's output twist

`/local_planner/cmd_vel [geometry_msgs::Twist]:` Local planner's output twist

## Publishers

`/cmd_vel [geometry_msgs::Twist]:` Resultant blended/muxed command velocity that is sent to robot

## Parameters

`~/user_vel_topic:` Twist topic to subscribe to for user's joystick input

`~/shared_dwa_topic:` Twist topic to susbcribe to for shared_dwa's output velocity

`~/local_planner_topic:` Twist topic to subscribe to for local_planner's output velocity

`~/cmd_vel_topic:` Twist topic to publish the resultant velocity to

`~/rate:` Rate at which to run the multiplexer (Hz)

`~/blend_time:` Blending time window to fully switch from autonomous output velocity to shared control velocity (s)