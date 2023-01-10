# Nuturtle  Description
URDF files for Nuturtle Franklin the Turtles
* `ros2 launch nuturtle_description load_one.launch.py` to see the robot in rviz.
* `ros2 launch nuturtle_description load_all.launch.xml` to see four copies of the robot in rviz.

![image](https://user-images.githubusercontent.com/46512429/211421745-cc2239d8-01ed-4da6-ac96-ce4c56c5e53d.png)

* The rqt_graph when all four robots are visualized (Nodes Only, Hide Debug) is:

![rosgraphx](https://user-images.githubusercontent.com/46512429/211422346-1a173f85-52a7-4716-910e-a6fdc9550445.svg)

# Launch File Details
* Arguments of singular robot launch:
  `ros2 launch nuturtle_description load_one.launch.py --show-args`
  ```
  Arguments (pass arguments as '<name>:=<value>'):

      'color':
          Color of TurtleBot. Valid choices are: ['purple', 'red', 'green', 'blue']
          (default: 'purple')

      'use_rviz':
          Flag to enable rviz. Valid choices are: ['true', 'false']
          (default: 'true')

      'use_jsp':
          Flag to enable joint_state_publisher. Valid choices are: ['true', 'false']
          (default: 'true')
    ```

* Arguments of multiple robot launch:
  `ros2 launch nuturtle_description load_all.launch.xml --show-args`
  ```
  Arguments (pass arguments as '<name>:=<value>'):

    'color':
        Color of TurtleBot. Valid choices are: ['purple', 'red', 'green', 'blue']
        (default: 'purple')

    'use_rviz':
        Flag to enable rviz. Valid choices are: ['true', 'false']
        (default: 'true')

    'use_jsp':
        Flag to enable joint_state_publisher. Valid choices are: ['true', 'false']
        (default: 'true')
  ```

Worked With: Liz Metzger, Dilan Wijesinghe, Meg Sindelar, Marno Nel, Katie Hughes, Ava Zahedi, Nicolas Morales