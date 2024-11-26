from launch import LaunchDescription
from launch.substitutions import Command, PathJoinSubstitution, LaunchConfiguration, \
                                TextSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare, ExecutableInPackage
from launch.actions import DeclareLaunchArgument, Shutdown
from launch.actions import SetLaunchConfiguration
from launch.conditions import IfCondition
import os 
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    use_rviz = LaunchConfiguration('use_rviz')
    use_jsp = LaunchConfiguration('use_jsp')
    turtle_color = LaunchConfiguration('color')
    # rviz_config = LaunchConfiguration("rviz_config")

    # SetLaunchConfiguration(name='rviz_config',
    #                        value=[FindPackageShare("nuturtle_description"),
    #                               TextSubstitution(text='/config/basic_'),
    #                               turtle_color,
    #                               TextSubstitution(text='.rviz')])
    


    use_rviz_arg = DeclareLaunchArgument(
        name='use_rviz',
        default_value='True',
        choices=[
            'True',
            'False'],
        description='Flag to enable rviz')

    use_jsp_arg = DeclareLaunchArgument(
                name='use_jsp',
                default_value='True',
                choices=[
                    'True',
                    'False'],
                description='Flag to enable joint_state_publisher')

    turtle_color_arg = DeclareLaunchArgument(
                name='color',
                default_value='purple',
                choices=[
                    'purple',
                    'red',
                    'green',
                    'blue'],
                description='Color of TurtleBot')

    rviz_config = PathJoinSubstitution([
        FindPackageShare('nuturtle_description'),
        '/config/basic_',
        turtle_color,
        '.rviz'])

    robot_state_publisher_node = Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                namespace=turtle_color,
                parameters=[
                    {"frame_prefix":
                        PathJoinSubstitution([turtle_color, '']),
                     "robot_description":
                        Command([ExecutableInPackage("xacro", "xacro"), " ",
                                 PathJoinSubstitution(
                                    [FindPackageShare("nuturtle_description"),
                                        "urdf/turtlebot3_burger.urdf.xacro"]),
                                 " color:=",
                                 turtle_color])}],
                        )

    rviz_node = Node(
                package='rviz2',
                executable='rviz2',
                namespace=turtle_color,
                condition=IfCondition(
                            PythonExpression([
                                use_rviz, 
                            ]),
                ),
                name='rviz2',
                output='screen',
                arguments=['-d', rviz_config],
                on_exit=Shutdown()
                )

    jsp_node = Node(
                package='joint_state_publisher',
                executable='joint_state_publisher',
                namespace=turtle_color,
                condition=IfCondition(
                            PythonExpression([
                                use_jsp,
                            ]),
                ),
                )


    return LaunchDescription([
        use_rviz_arg,
        use_jsp_arg,
        turtle_color_arg,
        robot_state_publisher_node,
        rviz_node,
        jsp_node





        ])


