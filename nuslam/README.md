# NuSLAM

This package is responsible for performing EKF SLAM on the Turtlebot in both real life and simulation.

# Nodes
* `slam` - the node that runs EKF SLAM

# Launch File Details
* To launch EKF SLAM on the Turtlebot:
  `ros2 launch nuslam nuslam.launch.xml`
  ```
  Arguments (passed directly to node through `basic_world.yaml`):
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

      'input_noise':
          Input variance to add noise applied to wheel velocities (double).
          (default: '0.01')

      'slip_fraction':
          Slip fraction applied to wheels (double).
          (default: '0.05')

      'slam':
          Flag whether or not to run with SLAM (string).
          (default:  depends on robot arg)
   ```

# RVIZ after moving robot in a path while running SLAM
![image](https://user-images.githubusercontent.com/46512429/222614698-8bbb8e45-37c8-4cc9-bb67-8955d799e977.png)


Worked With: Liz Metzger, Dilan Wijesinghe, Meg Sindelar, Marno Nel, Katie Hughes, Ava Zahedi, Nicolas Morales