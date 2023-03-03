# NuSLAM

This package contains the simulator for Turtlebot SLAM. The `nusim` node which generates a
Turtlebot in an environment containing obstacles. The Turtlebot will navigate this enivroment and
perform Extended Kalman Filter (EKF) SLAM.

# Nodes
* `nusim` - main simulation node
* `basic_sensor.cpp` - publishes x,y cordinates of obstacles relative to the robot within a certain range
* `fake_laser_scan` - simulates the laser scan of the actual turtlebot and publishes fake laser scan messages

# Launch File Details
* To launch the Turtlebot in the simulator environment:
  `ros2 launch nusim nusim.launch.xml`
  ```
  Arguments (passed directly to node through `basic_world.yaml`):
      'x0':
          Initial X Position of Turtlebot (double).
          (default: '-0.6')

      'y0':
          Initial Y Position of Turtlebot (double).
          (default: '0.8')

      'theta0':
          Initial angular Position of Turtlebot in radians (double).
          (default: '1.57')

      'obstacles/x':
          List of X Positions of obstacles (std::vector<double>).
          (default: '[-0.6, 0.7, 0.5]')

      'obstacles/y':
          List of Y Positions of obstacles (std::vector<double>).
          (default: '[-0.8, -0.7, 0.9]')

      'obstacles/r':
          Radius of obstacles (double).
          (default: '0.038')
    
      'input_noise':
          Input variance to add noise applied to wheel velocities (double).
          (default: '0.01')

      'slip_fraction':
          Slip fraction applied to wheels (double).
          (default: '0.05')

      'basic_sensor_variance':
          Variance to change sensor noise for basic sensor (double).
          (default: '0.01')

      'max_range_basic':
          Maximum range for basic sensor (m) (double).
          (default: '0.75')

      'max_range_fake_laser':
          Maximum range for fake laser scanner (m) (double).
          (default: '3.5')

      'min_range_fake_laser':
          Minimum range for fake laser scanner (m) (double).
          (default: '0.11999999731779099')

      'angle_increment':
          The angle that the scanner is incrimenting by (double).
          (default: '0.01745329238474369')

      'noise_fake_laser':
          Variance for a zero mean noise applied to the fake laser scanner (double).
          (default: '0.01745329238474369')
   ```

# Defualt launch Rviz Simulation

![image](https://user-images.githubusercontent.com/46512429/213926452-eff151c0-1479-45d0-aa5f-249f8394d45f.png)


Worked With: Liz Metzger, Dilan Wijesinghe, Meg Sindelar, Marno Nel, Katie Hughes, Ava Zahedi, Nicolas Morales