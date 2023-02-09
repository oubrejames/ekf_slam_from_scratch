# ME495 Sensing, Navigation and Machine Learning For Robotics
* James Oubre
* Winter 2023
# Package List
This repository consists of several ROS packages
- nuturtle_description - contains robot description for Turtlebot and launch files to show in Rviz
- turtlelib - C++ library containing 2D transformation functions
- nusim - package to simulate environment and Turtlebot SLAM
- nuturtle_control - contains nodes to control the diff drive robot, keep track of its odometry, and
move the robot in predefined patterns
# Demonstration
### Video of robot moving in real world through turtle_control node and the corresponding odometry in RViz

[turtle_and_odom.webm](https://user-images.githubusercontent.com/46512429/217748731-2502167b-5606-4010-a3a2-ff0be5b51c05.webm)

In the format (x, y, theta) the original odometry position was (0, 0, 0). After driving the TurtleBot in a circle, changing the 
direction twice, stopping it, and then teleoperating the robot back to the starting position the ending
odometry reading was (0.15, 0.04, -0.0001818).
