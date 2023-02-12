# nuturtle_control

This package contains a number of nodes that are used in control of the TurtleBot in both simulation
and the real world. 

# Nodes
* `turtle_control` converts velocity commands to wheel motor commands and publishes them
    * Publishers: `wheel_cmd` - output wheel velocity (Dynamixel ticks - 1 tick = 0.024 rad/s)
    * Subcribers: `cmd_vel` - input velocity (rad/s)
* `odometry` calculates and publishes vehicle odometry
    * Publishers: `odom` - output odometry messages
    * Subcribers: `joint_states` - the robot's current joint states from its sensor readings
* `circle` publishes commands to drive the robot in a circle at a specified velocity and radius
    * Publishers: `cmd_vel` - twist correspinding to make robot move in a circle
    * Subcribers: None

# Launch File Details
* To launch the Turtlebot in the simulator environment:
  `ros2 launch nuturtle_control start_robot.launch`
  ```
  Arguments:
      'body_id':
          Name of robot's base frame (string).
          (default: '')

      'odom_id':
          Name of robot's odometry frame (string).
          (default: 'odom')

      'wheel_left':
          Name of robot's left wheel joint (string).
          (default: 'wheel_left_joint')

      'wheel_right':
          Name of robot's right wheel joint (string).
          (default: 'wheel_right_joint')

      'cmd_src':
          The source of commands for the robot (srting) (none, circle, teleop).
          (default: 'none')

      'robot':
          The source of the robot (string) (localhost, nusim, none).
          (default: 'nusim')

      'use_rviz':
          Flag for whether or not to use rviz.
          (default depends on `robot`)
    ```

# Services

* `initial_pose` sets the robot's odometry to an initial position
    * Type: nuturtle_control/srv/Spawn
    * Service call: `ros2 service call /initial_pose nuturtle_control/srv/Spawn "x: 0.0
y: 0.0
z: 0.0
theta: 0.0" 
`
* `control` moves the robot in a circle when called
    * Type: nuturtle_control/srv/Control
    * Service call: `ros2 service call /control nuturtle_control/srv/Control "velocity: 0.0
                    radius: 0.0" `
* `reverse` changes the robot's direction when driving in a circle
    * Type: std_srvs/srv/Empty
    * Service call: `ros2 service call /reverse std_srvs/srv/Empty`
* `stop` stops the robot by publishing a 0 command velocity
    * Type: std_srvs/srv/Empty
    * Service call: `ros2 service call /stop std_srvs/srv/Empty`

Worked With: Liz Metzger, Dilan Wijesinghe, Meg Sindelar, Marno Nel, Katie Hughes, Ava Zahedi, Nicolas Morales