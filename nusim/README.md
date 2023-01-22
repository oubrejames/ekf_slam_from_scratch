# Nusim

This package contains the simulator for Turtlebot SLAM. The `nusim` node which generates a
Turtlebot in an environment containing obstacles. The Turtlebot will navigate this enivroment and
perform Extended Kalman Filter (EKF) SLAM.

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
    ```

# Defualt launch Rviz Simulation

![image](https://user-images.githubusercontent.com/46512429/213926452-eff151c0-1479-45d0-aa5f-249f8394d45f.png)


Worked With: Liz Metzger, Dilan Wijesinghe, Meg Sindelar, Marno Nel, Katie Hughes, Ava Zahedi, Nicolas Morales